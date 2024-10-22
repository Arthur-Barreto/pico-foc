#include "consts.h"
#include <stdint.h>

const uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                              {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
const uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                              {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};

const float CONVERSION_FACTOR = 3.3f / (1 << 12);

volatile uint8_t timer_status = 0;
volatile uint8_t timer_currents_status = 0;
volatile uint8_t timer_fired = 0;
volatile uint8_t encoder_status = 0;
volatile uint8_t step_index = 0;
volatile float current_angle = 0.0;

float id_ref = 0.0;
float iq_ref = 0.5;

float kp = 0.05;
float ki = 200.0;

float id_error, iq_error;
float id_integrator = 0.0;
float iq_integrator = 0.0;
float id_output, iq_output;

float integrator_max = 10;