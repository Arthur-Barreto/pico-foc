#ifndef UTILS_H
#define UTILS_H

#include "consts.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

typedef struct {
  float cur_a;
  float cur_b;
} current_ab;

typedef struct {
  float cur_alpha;
  float cur_beta;
} current_clark;

typedef struct {
  float cur_d;
  float cur_q;
} current_park;

bool timer_0_callback(repeating_timer_t *rt);
bool timer_1_callback(repeating_timer_t *rt);
void encoder_callback(uint gpio, uint32_t events);
void align_rotor();
void move_clockwise();
current_ab get_current_ab();
current_clark get_clark_transform(current_ab cur_ab);
current_park get_park_transform(current_clark cur_clark);
current_clark get_inverse_park_transform(current_park cur_park);

#endif // UTILS_H