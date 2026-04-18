// ============================================================
// can_bus.cpp
// ============================================================
// Low-level TWAI (CAN) driver for ESP32 ↔ ODrive CAN Simple.
// ============================================================

#include "can_bus.h"
#include "config.h"
#include <Arduino.h>
#include "driver/twai.h"

// Single global feedback struct updated by can_bus_poll().
// All fields are written atomically on each CAN frame receipt;
// no mutex is needed because the ESP32 runs single-threaded here.
static MotorFeedback g_feedback = {0};

// ============================================================
// can_bus_init
// ============================================================
// Installs and starts the TWAI driver.  Must be called once
// from setup() before any transmit or poll functions.
//
// Returns: true on success, false if hardware init failed.
// ============================================================
bool can_bus_init() {
  // Build the general config from the macro helper, then override
  // queue depths to handle bursts from 6 simultaneous nodes.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 50;  // Enough for 6 nodes × multiple frame types
  g_config.tx_queue_len = 20;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  // Enable the alerts we care about for runtime diagnostics.
  twai_reconfigure_alerts(
      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF,
      NULL);

  return true;
}

// ============================================================
// can_bus_poll
// ============================================================
// Drain the entire TWAI RX queue in one call.  For each frame:
//   • Record the arrival time for the watchdog in safety.cpp.
//   • Decode heartbeat (0x01) and encoder estimate (0x09) frames.
//
// ODrive CAN Simple identifier format (11-bit):
//   bits [10:5] = node ID  (1–6)
//   bits  [4:0] = command ID
// ============================================================
void can_bus_poll() {
  twai_message_t rx_msg;

  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    uint32_t node_id = rx_msg.identifier >> 5;
    uint32_t cmd_id  = rx_msg.identifier & 0x1F;

    // Update the per-node arrival timestamp used by the safety watchdog.
    if (node_id >= 1 && node_id <= 6) {
      g_feedback.last_msg_time_ms[node_id] = millis();
    }

    // --- Heartbeat frame (cmd 0x01) ---
    // Bytes 0-3: axis_error (uint32)
    // Byte  4  : axis_state (uint8)
    if (cmd_id == 0x01) {
      uint32_t axis_error = 0;
      memcpy(&axis_error, &rx_msg.data[0], 4);

      switch (node_id) {
        case 1: g_feedback.left_wheel_error  = axis_error; break;
        case 2: g_feedback.right_wheel_error = axis_error; break;
        case 3: g_feedback.left_knee_error   = axis_error; break;
        case 4: g_feedback.right_knee_error  = axis_error; break;
        case 5: g_feedback.left_torso_error  = axis_error; break;
        case 6: g_feedback.right_torso_error = axis_error; break;
      }
    }

    // --- Encoder estimate frame (cmd 0x09) ---
    // Bytes 0-3: pos_estimate (float, motor revolutions)
    // Bytes 4-7: vel_estimate (float, motor rev/s)
    //
    // Convert from motor shaft to joint by dividing by GEAR_RATIO.
    else if (cmd_id == 0x09) {
      float pos_revs = 0.0f;
      float vel_revs = 0.0f;
      memcpy(&pos_revs, &rx_msg.data[0], 4);
      memcpy(&vel_revs, &rx_msg.data[4], 4);

      // Scale from motor revolutions to joint degrees.
      float pos_deg = (pos_revs / GEAR_RATIO) * 360.0f;
      float vel_dps = (vel_revs / GEAR_RATIO) * 360.0f;

      switch (node_id) {
        case 1: g_feedback.left_pos_deg        = pos_deg;
                g_feedback.left_vel_dps        = vel_dps; break;
        case 2: g_feedback.right_pos_deg       = pos_deg;
                g_feedback.right_vel_dps       = vel_dps; break;
        case 3: g_feedback.left_knee_pos_deg   = pos_deg; break;
        case 4: g_feedback.right_knee_pos_deg  = pos_deg; break;
        case 5: g_feedback.left_torso_pos_deg  = pos_deg; break;
        case 6: g_feedback.right_torso_pos_deg = pos_deg; break;
      }
    }
  }
}

// ============================================================
// can_bus_check_alerts
// ============================================================
// Read the TWAI alert register and print any active warnings.
// Call once per loop; the ESP32 auto-clears alerts on read.
// ============================================================
void can_bus_check_alerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
      Serial.println("WARNING: CAN RX QUEUE FULL! (Missed messages)");
    if (alerts & TWAI_ALERT_BUS_ERROR)
      Serial.println("WARNING: CAN BUS ERROR! (Check wires/resistors)");
    if (alerts & TWAI_ALERT_BUS_OFF)
      Serial.println("FATAL: CAN BUS OFF! (Severe electrical issue)");
  }
}

// ============================================================
// can_bus_transmit_frame
// ============================================================
// Low-level frame builder.  Assembles the 11-bit identifier from
// node_id and cmd_id and posts the frame to the TX queue.
// Timeout of 1 ms prevents blocking the control loop.
// ============================================================
void can_bus_transmit_frame(int node_id, int cmd_id, uint8_t *data, int len) {
  uint32_t full_id = (node_id << 5) | cmd_id;

  twai_message_t tx_msg;
  tx_msg.identifier      = full_id;
  tx_msg.extd            = 0;
  tx_msg.rtr             = 0;
  tx_msg.data_length_code = len;

  for (int i = 0; i < len; i++) tx_msg.data[i] = data[i];

  twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
}

// ============================================================
// Typed command helpers
// ============================================================

// Send velocity setpoint (ODrive cmd 0x0D).
// vel_value is in whatever unit the caller uses; typically deg/s
// at the joint (motors.cpp handles the deg→rev conversion).
void can_bus_send_velocity(int node_id, float vel_value) {
  uint8_t data[4];
  memcpy(data, &vel_value, sizeof(float));
  can_bus_transmit_frame(node_id, 0x0D, data, 4);
}

// Send torque setpoint (ODrive cmd 0x0E) in Nm at the rotor.
void can_bus_send_torque(int node_id, float torque_nm) {
  uint8_t data[4];
  memcpy(data, &torque_nm, sizeof(float));
  can_bus_transmit_frame(node_id, 0x0E, data, 4);
}

// Send position setpoint (ODrive cmd 0x0C) in motor revolutions.
// The 8-byte frame format: bytes 0-3 = position (float),
// bytes 4-7 = velocity feed-forward (float, zero here).
void can_bus_send_position(int node_id, float pos_revs) {
  uint8_t data[8] = {0};
  memcpy(data, &pos_revs, sizeof(float));
  can_bus_transmit_frame(node_id, 0x0C, data, 8);
}

// Return a copy of the latest decoded feedback.
MotorFeedback can_bus_get_feedback() {
  return g_feedback;
}
