#ifndef CONSTS_H
#define CONSTS_H

#include <stdint.h>

// Pin definitions
#define IN1 2
#define IN2 3
#define IN3 4
#define EN1 5
#define EN2 6
#define EN3 7
#define ENCODER 16
#define CUR_A 26
#define CUR_B 27

// Conversion factor for ADC (3.3V / 12-bit resolution)
extern const float CONVERSION_FACTOR;

// Motor control sequences
extern const uint8_t in_seq[6][3];
extern const uint8_t en_seq[6][3];

// Volatile variables that are shared between main loop and ISR
extern volatile uint8_t timer_status;
extern volatile uint8_t timer_currents_status;
extern volatile uint8_t encoder_status;
extern volatile uint8_t step_index;

// Angle of the current (used in FOC or other control algorithms)
extern volatile float current_angle;

#endif // CONSTS_H
