#include "ros_bridge.h"
#include "config.h"
#include "can_bus.h"
#include "utils.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Serial port selection
// Define ROS_SERIAL_PORT in config.h to override (e.g. Serial2).
// Default: Serial  (USB-CDC / UART0)
// ---------------------------------------------------------------------------
#ifndef ROS_SERIAL_PORT
  #define ROS_SERIAL_PORT Serial
#endif

namespace
{
constexpr uint8_t HEADER_1 = 0xAA;
constexpr uint8_t HEADER_2 = 0x55;
constexpr uint8_t MSG_WHEEL_VELOCITY = 0x01;
constexpr uint8_t MSG_MOTOR_ENABLE = 0x02;
constexpr uint8_t MSG_CONTROL_MODE = 0x03;
constexpr uint8_t MSG_FULL_STATE = 0x12;
constexpr uint8_t MAX_PAYLOAD = 64;

#ifndef ROS_BRIDGE_STATE_PERIOD_MS
constexpr unsigned long STATE_PERIOD_MS = 10;  // 100 Hz default feedback stream
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

float s_forward_cmd = 0.0f;
float s_turn_cmd = 0.0f;
bool s_has_ros_cmd = false;
bool s_motor_enabled = false;
uint8_t s_control_mode = 0;

float s_left_cmd_rad_s = 0.0f;
float s_right_cmd_rad_s = 0.0f;

unsigned long s_last_cmd_ms = 0;
unsigned long s_timeout_ms = ROS_CMD_TIMEOUT_MS;
unsigned long s_last_state_tx_ms = 0;
bool s_reset_requested = false;

#if ROS_BRIDGE_DEMO_FEEDBACK
float s_demo_left_pos_deg = 0.0f;
float s_demo_right_pos_deg = 0.0f;
float s_demo_left_vel_dps = 0.0f;
float s_demo_right_vel_dps = 0.0f;
unsigned long s_demo_last_update_ms = 0;
#endif

uint8_t checksum(uint8_t msg_id, uint8_t len, const uint8_t *payload)
{
  uint8_t sum = static_cast<uint8_t>(msg_id + len);
  for (uint8_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + payload[i]);
  }
  return sum;
}

void reset_parser()
{
  s_parser_state = ParseState::WAIT_HEADER_1;
  s_rx_msg_id = 0;
  s_rx_len = 0;
  s_rx_checksum = 0;
  s_rx_index = 0;
}

template<typename T>
bool read_payload(const uint8_t *payload, uint8_t len, uint8_t &offset, T &value)
{
  if (static_cast<uint8_t>(offset + sizeof(T)) > len) {
    return false;
  }
  memcpy(&value, payload + offset, sizeof(T));
  offset = static_cast<uint8_t>(offset + sizeof(T));
  return true;
}

template<typename T>
void write_payload(uint8_t *payload, uint8_t &offset, const T &value)
{
  memcpy(payload + offset, &value, sizeof(T));
  offset = static_cast<uint8_t>(offset + sizeof(T));
}

bool send_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
  if (len > MAX_PAYLOAD) {
    return false;
  }

  // Fixed-size packet: [AA][55][msg_id][len][payload 0..len-1][checksum]
  uint8_t packet[2 + 1 + 1 + MAX_PAYLOAD + 1];
  uint8_t offset = 0;
  packet[offset++] = HEADER_1;
  packet[offset++] = HEADER_2;
  packet[offset++] = msg_id;
  packet[offset++] = len;
  for (uint8_t i = 0; i < len; ++i) {
    packet[offset++] = payload[i];
  }
  packet[offset++] = checksum(msg_id, len, payload);

  // Write the full assembled packet (headers included) via the configured port.
  return ROS_SERIAL_PORT.write(packet, offset) == offset;
}

#if ROS_BRIDGE_DEMO_FEEDBACK
void update_demo_feedback()
{
  unsigned long now = millis();
  if (s_demo_last_update_ms == 0) {
    s_demo_last_update_ms = now;
    return;
  }

  float dt = (now - s_demo_last_update_ms) * 0.001f;
  s_demo_last_update_ms = now;
  if (dt <= 0.0f) {
    return;
  }

  s_demo_left_vel_dps = rad2deg(s_left_cmd_rad_s);
  s_demo_right_vel_dps = rad2deg(s_right_cmd_rad_s);
  s_demo_left_pos_deg += s_demo_left_vel_dps * dt;
  s_demo_right_pos_deg += s_demo_right_vel_dps * dt;
}

MotorFeedback get_demo_feedback()
{
  MotorFeedback fb{};
  fb.left_pos_deg = s_demo_left_pos_deg;
  fb.right_pos_deg = s_demo_right_pos_deg;
  fb.left_vel_dps = s_demo_left_vel_dps;
  fb.right_vel_dps = s_demo_right_vel_dps;
  return fb;
}
#endif

void apply_wheel_targets_rad_s(float left_rad_s, float right_rad_s)
{
  s_left_cmd_rad_s = left_rad_s;
  s_right_cmd_rad_s = right_rad_s;

  float avg_rad_s = 0.5f * (left_rad_s + right_rad_s);
  float diff_rad_s = 0.5f * (right_rad_s - left_rad_s);

  float forward = avg_rad_s / ROS_BRIDGE_MAX_WHEEL_RAD_S;
  float turn = diff_rad_s / ROS_BRIDGE_MAX_WHEEL_RAD_S;

  forward = clampf(forward, -1.0f, 1.0f);
  turn = clampf(turn, -1.0f, 1.0f);
  if (fabs(forward) < ROS_BRIDGE_CMD_DEADBAND) forward = 0.0f;
  if (fabs(turn) < ROS_BRIDGE_CMD_DEADBAND) turn = 0.0f;

  s_forward_cmd = forward;
  s_turn_cmd = turn;
  s_has_ros_cmd = true;
  s_last_cmd_ms = millis();
}

void handle_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
  uint8_t offset = 0;

  switch (msg_id) {
    case MSG_WHEEL_VELOCITY: {
      float left_rad_s = 0.0f;
      float right_rad_s = 0.0f;
      if (len != 8 || !read_payload(payload, len, offset, left_rad_s) ||
          !read_payload(payload, len, offset, right_rad_s)) {
        return;
      }
      apply_wheel_targets_rad_s(left_rad_s, right_rad_s);
      break;
    }

    case MSG_MOTOR_ENABLE: {
      if (len != 1) {
        return;
      }
      s_motor_enabled = payload[0] != 0;
      if (!s_motor_enabled) {
        apply_wheel_targets_rad_s(0.0f, 0.0f);
        s_has_ros_cmd = false;
      }
      break;
    }

    case MSG_CONTROL_MODE: {
      if (len != 1) {
        return;
      }
      s_control_mode = payload[0];
      break;
    }

    default:
      break;
  }
}

void parse_byte(uint8_t byte)
{
  switch (s_parser_state) {
    case ParseState::WAIT_HEADER_1:
      if (byte == HEADER_1) {
        s_parser_state = ParseState::WAIT_HEADER_2;
      }
      break;

    case ParseState::WAIT_HEADER_2:
      if (byte == HEADER_2) {
        s_parser_state = ParseState::READ_MSG_ID;
      } else if (byte != HEADER_1) {
        s_parser_state = ParseState::WAIT_HEADER_1;
      }
      break;

    case ParseState::READ_MSG_ID:
      s_rx_msg_id = byte;
      s_rx_checksum = byte;
      s_parser_state = ParseState::READ_LEN;
      break;

    case ParseState::READ_LEN:
      s_rx_len = byte;
      s_rx_checksum = static_cast<uint8_t>(s_rx_checksum + byte);
      s_rx_index = 0;
      if (s_rx_len > MAX_PAYLOAD) {
        reset_parser();
      } else if (s_rx_len == 0) {
        s_parser_state = ParseState::READ_CHECKSUM;
      } else {
        s_parser_state = ParseState::READ_PAYLOAD;
      }
      break;

    case ParseState::READ_PAYLOAD:
      s_rx_payload[s_rx_index++] = byte;
      s_rx_checksum = static_cast<uint8_t>(s_rx_checksum + byte);
      if (s_rx_index >= s_rx_len) {
        s_parser_state = ParseState::READ_CHECKSUM;
      }
      break;

    case ParseState::READ_CHECKSUM:
      if (byte == s_rx_checksum) {
        handle_packet(s_rx_msg_id, s_rx_payload, s_rx_len);
      }
      reset_parser();
      break;
  }
}

void maybe_send_periodic_state()
{
  const unsigned long now = millis();
  if ((now - s_last_state_tx_ms) < STATE_PERIOD_MS) {
    return;
  }
  s_last_state_tx_ms = now;

#if ROS_BRIDGE_DEMO_FEEDBACK
  update_demo_feedback();
  MotorFeedback fb = get_demo_feedback();
#else
  MotorFeedback fb = can_bus_get_feedback();
#endif
  ros_bridge_publish_full_state(fb, 0.0f, 0.0f);
}
}  // namespace

bool ros_bridge_init(unsigned long baudrate)
{
  ROS_SERIAL_PORT.begin(baudrate);
  delay(200);

  s_last_cmd_ms = millis();
  s_last_state_tx_ms = millis();
  reset_parser();

#if ROS_BRIDGE_DEMO_FEEDBACK
  s_demo_last_update_ms = millis();
  s_demo_left_pos_deg = 0.0f;
  s_demo_right_pos_deg = 0.0f;
  s_demo_left_vel_dps = 0.0f;
  s_demo_right_vel_dps = 0.0f;
#endif

  return true;
}

void ros_bridge_set_timeout_ms(unsigned long timeout_ms)
{
  s_timeout_ms = timeout_ms;
}

void ros_bridge_update()
{
#if ROS_BRIDGE_DEMO_FEEDBACK
  update_demo_feedback();
#endif

  while (ROS_SERIAL_PORT.available() > 0) {
    parse_byte(static_cast<uint8_t>(ROS_SERIAL_PORT.read()));
  }

  if ((millis() - s_last_cmd_ms) > s_timeout_ms) {
    s_forward_cmd = 0.0f;
    s_turn_cmd = 0.0f;
    s_has_ros_cmd = false;
    s_left_cmd_rad_s = 0.0f;
    s_right_cmd_rad_s = 0.0f;
  }

  maybe_send_periodic_state();
}

void ros_bridge_get_commands(float &forward_cmd, float &turn_cmd, bool &has_ros_cmd)
{
  forward_cmd = s_motor_enabled ? s_forward_cmd : 0.0f;
  turn_cmd = s_motor_enabled ? s_turn_cmd : 0.0f;
  has_ros_cmd = s_motor_enabled && s_has_ros_cmd;
}

bool ros_bridge_reset_requested()
{
  return s_reset_requested;
}

void ros_bridge_clear_reset_request()
{
  s_reset_requested = false;
}

bool ros_bridge_motor_enabled()
{
  return s_motor_enabled;
}

uint8_t ros_bridge_control_mode()
{
  return s_control_mode;
}

float ros_bridge_left_target_rad_s()
{
  return s_left_cmd_rad_s;
}

float ros_bridge_right_target_rad_s()
{
  return s_right_cmd_rad_s;
}

void ros_bridge_publish_feedback(const MotorFeedback &fb)
{
  ros_bridge_publish_full_state(fb, 0.0f, 0.0f);
}

void ros_bridge_publish_full_state(const MotorFeedback &fb, float pitch, float gyro_y)
{
  uint8_t payload[4 + 6 * sizeof(float)];
  uint8_t offset = 0;

  const uint32_t time_ms = millis();
  const float left_pos = deg2rad(fb.left_pos_deg);
  const float right_pos = deg2rad(fb.right_pos_deg);
  const float left_vel = deg2rad(fb.left_vel_dps);
  const float right_vel = deg2rad(fb.right_vel_dps);

  write_payload(payload, offset, time_ms);
  write_payload(payload, offset, left_pos);
  write_payload(payload, offset, right_pos);
  write_payload(payload, offset, left_vel);
  write_payload(payload, offset, right_vel);
  write_payload(payload, offset, pitch);
  write_payload(payload, offset, gyro_y);

  send_packet(MSG_FULL_STATE, payload, offset);
}