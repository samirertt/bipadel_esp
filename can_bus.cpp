#include "can_bus.h"
#include "config.h"

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "driver/twai.h"

static MCP_CAN CAN0(MCP_CS_PIN);
static MotorFeedback g_feedback = {0};

bool can_bus_init() {
  // TWAI - motor 1
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 20;
  g_config.tx_queue_len = 20;

  twai_timing_config_t t_config = BAUD_RATE_NATIVE;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    return false;
  }

  // MCP2515 - motor 2
  if (CAN0.begin(MCP_ANY, BAUD_RATE_MCP, MCP_8MHZ) != CAN_OK) {
    return false;
  }
  CAN0.setMode(MCP_NORMAL);

  return true;
}

void can_bus_poll() {
  // Bus B - motor 2
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if ((rxId & 0x1F) == 0x09) {
      memcpy(&g_feedback.right_pos_deg, &rxBuf[0], 4);
      memcpy(&g_feedback.right_vel_dps, &rxBuf[4], 4);
    }
  }

  // Bus A - motor 1
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if ((rx_msg.identifier & 0x1F) == 0x09) {
      memcpy(&g_feedback.left_pos_deg, &rx_msg.data[0], 4);
      memcpy(&g_feedback.left_vel_dps, &rx_msg.data[4], 4);
    }
  }
}

void can_bus_transmit_frame(int node_id, int cmd_id, uint8_t* data, int len) {
  uint32_t full_id = (node_id << 5) | cmd_id;

  if (node_id == 1) {
    twai_message_t msg = {};
    msg.identifier = full_id;
    msg.extd = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, data, len);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
  } else {
    CAN0.sendMsgBuf(full_id, 0, 8, data);
  }
}

void can_bus_send_velocity(int node_id, float vel_deg_s) {
  uint8_t d[8] = {0};
  memcpy(d, &vel_deg_s, 4);
  can_bus_transmit_frame(node_id, 0x0D, d, 8);
}

void can_bus_send_torque(int node_id, float torque_nm) {
  uint8_t d[8] = {0};
  memcpy(d, &torque_nm, 4);
  can_bus_transmit_frame(node_id, 0x0E, d, 8);
}

MotorFeedback can_bus_get_feedback() {
  return g_feedback;
}