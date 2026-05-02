// ============================================================
// can_bus.cpp
// ============================================================
// Low-level TWAI (CAN) driver for ESP32 <-> ODrive CAN Simple.
// ============================================================

#include "can_bus.h"
#include "config.h"
#include <Arduino.h>
#include "driver/twai.h"

// Single global feedback struct updated by can_bus_poll().
static MotorFeedback g_feedback = {0};

// ============================================================
// can_bus_init
// ============================================================
bool can_bus_init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 50;  // Enough for 6 nodes x multiple frame types
  g_config.tx_queue_len = 20;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  twai_reconfigure_alerts(
      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF,
      NULL);

  return true;
}

// ============================================================
// can_bus_poll
// ============================================================
void can_bus_poll() {
  twai_message_t rx_msg;

  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    uint32_t node_id = rx_msg.identifier >> 5;
    uint32_t cmd_id  = rx_msg.identifier & 0x1F;

    if (node_id >= 1 && node_id <= 6) {
      g_feedback.last_msg_time_ms[node_id] = millis();
    }

    // --- Heartbeat frame (cmd 0x01) ---
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
    else if (cmd_id == 0x09) {
      float pos_revs = 0.0f;
      float vel_revs = 0.0f;
      memcpy(&pos_revs, &rx_msg.data[0], 4);
      memcpy(&vel_revs, &rx_msg.data[4], 4);

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

void can_bus_send_velocity(int node_id, float vel_value) {
  uint8_t data[4];
  memcpy(data, &vel_value, sizeof(float));
  can_bus_transmit_frame(node_id, 0x0D, data, 4);
}

void can_bus_send_torque(int node_id, float torque_nm) {
  uint8_t data[4];
  memcpy(data, &torque_nm, sizeof(float));
  can_bus_transmit_frame(node_id, 0x0E, data, 4);
}

// --- UPDATED: Send Position with Vel_FF and Torque_FF ---
void can_bus_send_position(int node_id, float pos_revs, float vel_ff_rps, float torque_ff_nm) {
  
  // Safety Clamps: Prevent Int16 overflow (Max 32.7 Nm)
  // Constrain torque to safe limits for testing
  if (torque_ff_nm > 10.0f) torque_ff_nm = 10.0f;
  if (torque_ff_nm < -10.0f) torque_ff_nm = -10.0f;

  uint8_t data[8] = {0};
  
  // Byte 0-3: Position (Float32)
  memcpy(data, &pos_revs, 4);
  
  // Byte 4-5: Velocity Feedforward (Int16, 0.001 scale)
  int16_t vel_ff_encoded = (int16_t)(vel_ff_rps * 1000.0f);
  memcpy(data + 4, &vel_ff_encoded, 2);
  
  // Byte 6-7: Torque Feedforward (Int16, 0.001 scale)
  int16_t torque_ff_encoded = (int16_t)(torque_ff_nm * 1000.0f);
  memcpy(data + 6, &torque_ff_encoded, 2);
  
  can_bus_transmit_frame(node_id, 0x0C, data, 8);
}

// Return a copy of the latest decoded feedback.
MotorFeedback can_bus_get_feedback() {
  return g_feedback;
}