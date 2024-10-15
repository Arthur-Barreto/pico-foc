#include "consts.h"
#include <stdint.h>

const uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                              {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
const uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                              {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};

const float CONVERSION_FACTOR = 3.3f / (1 << 12);

volatile uint8_t timer_status = 0;
volatile uint8_t timer_currents_status = 0;
volatile uint8_t encoder_status = 0;
volatile uint8_t step_index = 0;
volatile float current_angle = 0.0;
