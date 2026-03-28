#pragma once

void command_filter_init();
float command_filter_limit(float target, float previous, float max_step);