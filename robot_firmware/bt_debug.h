#pragma once 
#include <Arduino.h>

bool bt_debug_init();
void bt_debug_printf(const char* format,...);
void bt_debug_update();