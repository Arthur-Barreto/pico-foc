#include "utils.h"

// Timer callback to control motor movement
bool timer_0_callback(repeating_timer_t *rt) {
  timer_status = 1;
  return true;
}

// timer to update currents values
bool timer_1_callback(repeating_timer_t *rt) {
  timer_currents_status = 1;
  return true;
}

// Encoder interrupt callback
void encoder_callback(uint gpio, uint32_t events) {
  if (gpio == ENCODER) {
    encoder_status = 1;
  }
}

// Function to align rotor to a known position
void align_rotor() {
  // Set the motor to a known position
  gpio_put(EN1, 0);
  gpio_put(IN1, 0); // Set first winding

  gpio_put(EN2, 1);
  gpio_put(IN2, 1); // Set second winding

  gpio_put(EN3, 1);
  gpio_put(IN3, 0); // Disable third winding

  current_angle = 330.0; // Assume rotor is now aligned to 330 degrees

  printf("Rotor aligned at 330 degrees.\n");

  busy_wait_ms(500); // Give time for the rotor to stabilize

  // Disable all windings after alignment
  gpio_put(EN1, 0);
  gpio_put(EN2, 0);
  gpio_put(EN3, 0);
}

// Motor movement function based on the timer flag
void move_clockwise() {
  if (timer_status) {
    // Set motor pins based on the current step
    gpio_put(IN1, in_seq[step_index][0]);
    gpio_put(IN2, in_seq[step_index][1]);
    gpio_put(IN3, in_seq[step_index][2]);

    gpio_put(EN1, en_seq[step_index][0]);
    gpio_put(EN2, en_seq[step_index][1]);
    gpio_put(EN3, en_seq[step_index][2]);

    // Increment and wrap around the step index
    step_index = (step_index + 1) % 6;

    // Reset the timer flag
    timer_status = 0;
  }
}

current_ab get_current_ab() {
  current_ab res;
  adc_select_input(0);
  res.cur_a = adc_read() * CONVERSION_FACTOR;
  adc_select_input(1);
  res.cur_b = adc_read() * CONVERSION_FACTOR;
  return res;
}

current_clark get_clark_transform(current_ab cur_ab) {
  current_clark res;
  res.cur_alpha = cur_ab.cur_a;
  res.cur_beta = cur_ab.cur_a / sqrt(3) + 2 * cur_ab.cur_b / sqrt(3);
  return res;
}

current_park get_park_transform(current_clark cur_clark) {
  current_park res;
  res.cur_d = cos(current_angle) * cur_clark.cur_alpha +
              sin(current_angle) * cur_clark.cur_beta;
  res.cur_q = -sin(current_angle) * cur_clark.cur_alpha +
              cos(current_angle) * cur_clark.cur_beta;
  return res;
}