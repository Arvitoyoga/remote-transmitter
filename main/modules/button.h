#ifndef BUTTON_H
#define BUTTON_H

#include "driver/gpio.h"

#define MAX_BUTTONS 16

typedef void (*button_callback_t)(int id, int state);

void button_init(void);
void button_register(gpio_num_t pin);
int button_count_get(void);
bool button_get_state(int id);
bool button_check_hold_release(int id, int hold_time_ms, int reset_trig);
bool button_toggle_delay(int id, int active_time_ms);
bool button_toggle(int id);

#endif
