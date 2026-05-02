#include "bt_debug.h"
#include <BluetoothSerial.h>
#include "config.h"

static BluetoothSerial SerialBT;

bool bt_debug_init() {
  SerialBT.begin(BT_DEVICE_NAME);  // Bluetooth device name
  return true;
}

void bt_debug_printf(const char* format, ...) {
  if (!SerialBT.hasClient()) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  SerialBT.print(buffer);
  // Also mirror to USB Serial for convenience
  Serial.print(buffer);
}

void bt_debug_update() {
  // Optional: handle any BT events if needed
}