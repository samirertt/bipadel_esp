#include "ros_bridge.h"
#include "config.h"
#include "can_bus.h"
#include "utils.h"
#include "imu.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Serial port selection
// ---------------------------------------------------------------------------
#ifndef ROS_SERIAL_PORT
  #define ROS_SERIAL_PORT Serial
#endif

namespace
{
constexpr uint8_t HEADER_1 = 0xAA;
constexpr uint8_t HEADER_2 = 0x55;
constexpr uint8_t MSG_WHEEL_VELOCITY = 0x01;
constexpr uint8_t MSG_MOTOR_ENABLE   = 0x02;
constexpr uint8_t MSG_CONTROL_MODE   = 0x03;
// 0x05: ESP32 → ROS — tells the host to DTR-reset the ESP32.
// Sent once when all 6 motors first become alive after a cold boot.
constexpr uint8_t MSG_ESP_RESET      = 0x05;
constexpr uint8_t MSG_FULL_STATE     = 0x12;

constexpr uint8_t MAX_PAYLOAD = 64;

#ifndef ROS_BRIDGE_STATE_PERIOD_MS
constexpr unsigned long STATE_PERIOD_MS = 10;
#else
constexpr unsigned long STATE_PERIOD_MS = ROS_BRIDGE_STATE_PERIOD_MS;
#endif

enum class ParseState : uint8_t
{
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  READ_MSG_ID,
  READ_LEN,
  READ_PAYLOAD,
  READ_CHECKSUM
};

ParseState s_parser_state = ParseState::WAIT_HEADER_1;

uint8_t s_rx_msg_id = 0;
uint8_t s_rx_len = 0;
uint8_t s_rx_checksum = 0;
uint8_t s_rx_index = 0;
uint8_t s_rx_payload[MAX_PAYLOAD];

// Commands
float s_forward_cmd = 0.0f;
float s_turn_cmd = 0.0f;
bool  s_has_ros_cmd = false;
#if ROS_BRIDGE_TEST_MODE
bool s_motor_enabled = true;   // ← trust ROS; no separate enable needed
#else
bool s_motor_enabled = false;
#endif
uint8_t s_control_mode = 0;

float s_left_cmd_rad_s  = 0.0f;
float s_right_cmd_rad_s = 0.0f;

// Timing
unsigned long s_last_cmd_ms = 0;
unsigned long s_timeout_ms  = ROS_CMD_TIMEOUT_MS;
unsigned long s_last_state_tx_ms = 0;

bool s_reset_requested = false;

// ------------------------------------------------
// Utilities
// ------------------------------------------------

uint8_t checksum(uint8_t msg_id, uint8_t len, const uint8_t *payload)
{
  uint8_t sum = msg_id + len;
  for (uint8_t i = 0; i < len && payload != nullptr; i++) {
    sum += payload[i];
  }
  return sum;
}

void reset_parser()
{
  s_parser_state = ParseState::WAIT_HEADER_1;
  s_rx_index = 0;
}

template<typename T>
bool read_payload(const uint8_t *payload, uint8_t len, uint8_t &offset, T &value)
{
  if (offset + sizeof(T) > len) return false;
  memcpy(&value, payload + offset, sizeof(T));
  offset += sizeof(T);
  return true;
}

template<typename T>
void write_payload(uint8_t *payload, uint8_t &offset, const T &value)
{
  memcpy(payload + offset, &value, sizeof(T));
  offset += sizeof(T);
}

bool send_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
  uint8_t packet[2 + 1 + 1 + MAX_PAYLOAD + 1];
  uint8_t offset = 0;

  packet[offset++] = HEADER_1;
  packet[offset++] = HEADER_2;
  packet[offset++] = msg_id;
  packet[offset++] = len;

  for (uint8_t i = 0; i < len; i++) {
    packet[offset++] = payload[i];  // payload is null only when len==0, so safe
  }

  packet[offset++] = checksum(msg_id, len, payload);

  return ROS_SERIAL_PORT.write(packet, offset) == offset;
}

// ------------------------------------------------
// Command handling
// ------------------------------------------------

void apply_wheel_targets_rad_s(float left, float right)
{
  s_left_cmd_rad_s  = left;
  s_right_cmd_rad_s = right;

  float avg  = 0.5f * (left + right);
  float diff = 0.5f * (right - left);

  s_forward_cmd = clampf(avg / ROS_BRIDGE_MAX_WHEEL_RAD_S, -1.0f, 1.0f);
  s_turn_cmd    = clampf(diff / ROS_BRIDGE_MAX_WHEEL_RAD_S, -1.0f, 1.0f);

  s_has_ros_cmd = true;
  s_last_cmd_ms = millis();
}

void handle_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
  uint8_t offset = 0;

  if (msg_id == MSG_WHEEL_VELOCITY && len == 8) {
    float l, r;
    if (read_payload(payload, len, offset, l) &&
        read_payload(payload, len, offset, r)) {
      apply_wheel_targets_rad_s(l, r);
    }
  }
  else if (msg_id == MSG_MOTOR_ENABLE && len == 1) {
    s_motor_enabled = payload[0];
  }
  else if (msg_id == MSG_CONTROL_MODE && len == 1) {
    s_control_mode = payload[0];
  }
}

// ------------------------------------------------
// Parser
// ------------------------------------------------

void parse_byte(uint8_t b)
{
  switch (s_parser_state) {
    case ParseState::WAIT_HEADER_1:
      if (b == HEADER_1) s_parser_state = ParseState::WAIT_HEADER_2;
      break;

    case ParseState::WAIT_HEADER_2:
      if (b == HEADER_2) s_parser_state = ParseState::READ_MSG_ID;
      else s_parser_state = ParseState::WAIT_HEADER_1;
      break;

    case ParseState::READ_MSG_ID:
      s_rx_msg_id = b;
      s_rx_checksum = b;
      s_parser_state = ParseState::READ_LEN;
      break;

    case ParseState::READ_LEN:
      s_rx_len = b;
      s_rx_checksum += b;
      s_rx_index = 0;
      s_parser_state = (s_rx_len == 0) ? ParseState::READ_CHECKSUM : ParseState::READ_PAYLOAD;
      break;

    case ParseState::READ_PAYLOAD:
      s_rx_payload[s_rx_index++] = b;
      s_rx_checksum += b;
      if (s_rx_index >= s_rx_len)
        s_parser_state = ParseState::READ_CHECKSUM;
      break;

    case ParseState::READ_CHECKSUM:
      if (b == s_rx_checksum) {
        handle_packet(s_rx_msg_id, s_rx_payload, s_rx_len);
      }
      reset_parser();
      break;
  }
}

// ------------------------------------------------
// Periodic full-state sender
// ------------------------------------------------

void maybe_send_periodic_state()
{
  unsigned long now = millis();
  if (now - s_last_state_tx_ms < STATE_PERIOD_MS) return;
  s_last_state_tx_ms = now;

  MotorFeedback fb = can_bus_get_feedback();
  ImuData imu = imu_get_data();

  uint8_t payload[4 + 10 * sizeof(float) + 2];
  uint8_t offset = 0;

  uint32_t t = (uint32_t)millis();

  write_payload(payload, offset, t);

  write_payload(payload, offset, deg2rad(fb.left_pos_deg));
  write_payload(payload, offset, deg2rad(fb.right_pos_deg));
  write_payload(payload, offset, deg2rad(fb.left_vel_dps));
  write_payload(payload, offset, deg2rad(fb.right_vel_dps));

  write_payload(payload, offset, imu.accel_x);
  write_payload(payload, offset, imu.accel_y);
  write_payload(payload, offset, imu.accel_z);

  write_payload(payload, offset, deg2rad(imu.gyro_x));
  write_payload(payload, offset, deg2rad(imu.gyro_y));
  write_payload(payload, offset, deg2rad(imu.gyro_z));

  // Motor status — same bitmask encoding as ros_bridge_publish_feedback
  uint8_t alive_mask = 0;
  uint8_t fault_mask = 0;
  for (int i = 1; i <= 6; i++) {
    if (fb.last_msg_time_ms[i] > 0 && (now - fb.last_msg_time_ms[i]) < 500)
      alive_mask |= static_cast<uint8_t>(1u << (i - 1));
  }
  if (fb.left_wheel_error)  fault_mask |= (1u << 0);
  if (fb.right_wheel_error) fault_mask |= (1u << 1);
  if (fb.left_knee_error)   fault_mask |= (1u << 2);
  if (fb.right_knee_error)  fault_mask |= (1u << 3);
  if (fb.left_torso_error)  fault_mask |= (1u << 4);
  if (fb.right_torso_error) fault_mask |= (1u << 5);

  payload[offset++] = alive_mask;
  payload[offset++] = fault_mask;

  send_packet(MSG_FULL_STATE, payload, offset);
}

} // namespace

// ------------------------------------------------
// PUBLIC API
// ------------------------------------------------

bool ros_bridge_init(unsigned long baudrate)
{
  ROS_SERIAL_PORT.begin(baudrate);
  delay(200);

  s_last_cmd_ms = millis();
  s_last_state_tx_ms = millis();
  reset_parser();

  return true;
}

void ros_bridge_update()
{
  while (ROS_SERIAL_PORT.available()) {
    parse_byte(ROS_SERIAL_PORT.read());
  }

  if (millis() - s_last_cmd_ms > s_timeout_ms) {
    s_forward_cmd = 0;
    s_turn_cmd = 0;
    s_has_ros_cmd = false;
  }

  maybe_send_periodic_state();
}

void ros_bridge_get_commands(float &fwd, float &turn, bool &valid)
{
  fwd   = s_motor_enabled ? s_forward_cmd : 0.0f;
  turn  = s_motor_enabled ? s_turn_cmd    : 0.0f;
  valid = s_motor_enabled && s_has_ros_cmd;
}

// ---------------------------------------------------------------------------
// ros_bridge_set_timeout_ms
// Sets how long (ms) without a command before the bridge zeros the outputs.
// ---------------------------------------------------------------------------
void ros_bridge_set_timeout_ms(unsigned long timeout_ms)
{
  s_timeout_ms = timeout_ms;
}

// ---------------------------------------------------------------------------
// ros_bridge_publish_feedback
// Immediate one-shot transmit of encoder + full IMU as a MSG_FULL_STATE packet.
// Called from the main loop when the sketch wants to push data on demand
// rather than waiting for the periodic timer.
// ---------------------------------------------------------------------------
void ros_bridge_publish_feedback(const MotorFeedback &fb, const ImuData &imu)
{
  // payload = 4-byte timestamp + 10 floats (wheel pos/vel, accel, gyro) + 2 status bytes
  uint8_t payload[4 + 10 * sizeof(float) + 2];
  uint8_t offset = 0;

  uint32_t t = (uint32_t)millis();

  write_payload(payload, offset, t);

  write_payload(payload, offset, deg2rad(fb.left_pos_deg));
  write_payload(payload, offset, deg2rad(fb.right_pos_deg));
  write_payload(payload, offset, deg2rad(fb.left_vel_dps));
  write_payload(payload, offset, deg2rad(fb.right_vel_dps));

  write_payload(payload, offset, imu.accel_x);
  write_payload(payload, offset, imu.accel_y);
  write_payload(payload, offset, imu.accel_z);

  write_payload(payload, offset, deg2rad(imu.gyro_x));
  write_payload(payload, offset, deg2rad(imu.gyro_y));
  write_payload(payload, offset, deg2rad(imu.gyro_z));

  // --- Motor status appended at the end of every FULL_STATE packet ---
  // alive_mask: bit (i-1) set when motor node i sent a heartbeat within 500 ms.
  // fault_mask: bit order = LW, RW, LK, RK, LT, RT (matches node IDs 1-6).
  unsigned long now_ms = millis();
  uint8_t alive_mask = 0;
  uint8_t fault_mask = 0;
  for (int i = 1; i <= 6; i++) {
    if (fb.last_msg_time_ms[i] > 0 && (now_ms - fb.last_msg_time_ms[i]) < 500)
      alive_mask |= static_cast<uint8_t>(1u << (i - 1));
  }
  if (fb.left_wheel_error)  fault_mask |= (1u << 0);
  if (fb.right_wheel_error) fault_mask |= (1u << 1);
  if (fb.left_knee_error)   fault_mask |= (1u << 2);
  if (fb.right_knee_error)  fault_mask |= (1u << 3);
  if (fb.left_torso_error)  fault_mask |= (1u << 4);
  if (fb.right_torso_error) fault_mask |= (1u << 5);

  payload[offset++] = alive_mask;
  payload[offset++] = fault_mask;

  send_packet(MSG_FULL_STATE, payload, offset);
}

// ---------------------------------------------------------------------------
// ros_bridge_publish_full_state  (legacy / convenience wrapper)
// Kept for backwards compatibility with any call sites using pitch/gyro_y
// directly rather than a full ImuData struct.
// ---------------------------------------------------------------------------
void ros_bridge_publish_full_state(const MotorFeedback &fb, float pitch, float gyro_y)
{
  ImuData imu = {};
  imu.gyro_x         = pitch;   // pitch rate on X axis
  imu.gyro_y         = gyro_y;
  imu.fused_angle_deg = pitch;
  ros_bridge_publish_feedback(fb, imu);
}

// ---------------------------------------------------------------------------
// Remaining public accessors declared in ros_bridge.h
// ---------------------------------------------------------------------------

bool ros_bridge_reset_requested()       { return s_reset_requested; }
void ros_bridge_clear_reset_request()   { s_reset_requested = false; }
bool ros_bridge_motor_enabled()         { return s_motor_enabled; }
uint8_t ros_bridge_control_mode()       { return s_control_mode; }
float ros_bridge_left_target_rad_s()    { return s_left_cmd_rad_s; }
float ros_bridge_right_target_rad_s()   { return s_right_cmd_rad_s; }

// ---------------------------------------------------------------------------
// ros_bridge_notify_ros_reset
// Sends MSG_ESP_RESET (0x05, zero-byte payload) to the ROS hardware interface.
// The host receives this and performs a DTR-line reset so the ESP32 reboots
// cleanly with all motors already powered.  Call this ONCE when all 6 motors
// first become alive after a cold boot where motors were off.
// ---------------------------------------------------------------------------
void ros_bridge_notify_ros_reset()
{
  // Zero-byte payload: [AA][55][05][00][checksum]
  // checksum = msg_id + len + (no payload bytes) = 0x05 + 0x00 = 0x05
  send_packet(MSG_ESP_RESET, nullptr, 0);
}