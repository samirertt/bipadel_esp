#include "can_bus.h"
#include "config.h"

#include <Arduino.h>
#include "driver/twai.h"

static MotorFeedback g_feedback = {0};

bool can_bus_init() {
  // Initialize native TWAI Configuration for BOTH motors on the same bus
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  
  g_config.rx_queue_len = 20;
  g_config.tx_queue_len = 20;

  // SteadyWin motors default to 500Kbps
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI Driver install failed.");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI Driver start failed.");
    return false;
  }

  Serial.println("TWAI CAN Bus Initialized for Motors 1 & 2");
  return true;
}

void can_bus_poll() {
  twai_message_t rx_msg;
  
  // Read all available messages in the TWAI queue
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    // Standard 11-bit ID structure: Upper 6 bits = Node ID, Lower 5 bits = Command ID
    uint32_t node_id = rx_msg.identifier >> 5;
    uint32_t cmd_id = rx_msg.identifier & 0x1F;

    // Command 0x09: Motor Encoder Feedback (Position & Velocity)
    if (cmd_id == 0x09) {
      if (node_id == 1) { // Left motor
        memcpy(&g_feedback.left_pos_deg, &rx_msg.data[0], 4);
        memcpy(&g_feedback.left_vel_dps, &rx_msg.data[4], 4);
      } 
      else if (node_id == 2) { // Right motor
        memcpy(&g_feedback.right_pos_deg, &rx_msg.data[0], 4);
        memcpy(&g_feedback.right_vel_dps, &rx_msg.data[4], 4);
      }
    }
  }
}

void can_bus_transmit_frame(int node_id, int cmd_id, uint8_t* data, int len) {
  uint32_t full_id = (node_id << 5) | cmd_id;

  twai_message_t tx_msg;
  tx_msg.identifier = full_id;
  tx_msg.extd = 0; // Use Standard 11-bit ID
  tx_msg.rtr = 0;
  tx_msg.data_length_code = len;
  
  for (int i = 0; i < len; i++) {
    tx_msg.data[i] = data[i];
  }

  // Transmit on the shared bus. Max wait time is 1 tick so it doesn't block the LQR loop
  twai_transmit(&tx_msg, pdMS_TO_TICKS(1)); 
}

void can_bus_send_velocity(int node_id, float vel_deg_s) {
  uint8_t data[4];
  memcpy(data, &vel_deg_s, sizeof(float));
  // 0x0D is CMD_SET_INPUT_VELOCITY
  can_bus_transmit_frame(node_id, 0x0D, data, 4); 
}

void can_bus_send_torque(int node_id, float torque_nm) {
  uint8_t data[4];
  memcpy(data, &torque_nm, sizeof(float));
  // 0x0E is CMD_SET_INPUT_TORQUE
  can_bus_transmit_frame(node_id, 0x0E, data, 4); 
}

MotorFeedback can_bus_get_feedback() {
  return g_feedback;
}