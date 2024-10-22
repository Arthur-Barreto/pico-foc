#include "utils.h"

current_ab three_phase;
current_clark quadrature;
current_park rotated;
voltage_pi reference_voltage;
voltage_clark quadrature_voltage;
space_vector duty_cycle;
pwm_config_space_vector pwm_a, pwm_b, pwm_c;

int main() {
  stdio_init_all();
  sleep_ms(2000);

  // Configure the GPIOs for the motor
  gpio_init(EN1);
  gpio_init(EN2);
  gpio_init(EN3);

  gpio_set_dir(EN1, GPIO_OUT);
  gpio_set_dir(EN2, GPIO_OUT);
  gpio_set_dir(EN3, GPIO_OUT);

  // configure pwm for motor
  uint pwm_a_slice, pwm_a_chan;
  uint pwm_b_slice, pwm_b_chan;
  uint pwm_c_slice, pwm_c_chan;
  init_pwm(IN1, PWM_RES, &pwm_a_slice, &pwm_a_chan);
  init_pwm(IN2, PWM_RES, &pwm_b_slice, &pwm_b_chan);
  init_pwm(IN3, PWM_RES, &pwm_c_slice, &pwm_c_chan);

  pwm_a.slice_num = pwm_a_slice;
  pwm_a.chan_num = pwm_a_chan;
  pwm_b.slice_num = pwm_b_slice;
  pwm_b.chan_num = pwm_b_chan;
  pwm_c.slice_num = pwm_c_slice;
  pwm_c.chan_num = pwm_c_chan;

  // configure adc
  adc_init();
  adc_gpio_init(CUR_A);
  adc_gpio_init(CUR_B);

  // Configure the encoder GPIO
  gpio_init(ENCODER);
  gpio_set_dir(ENCODER, GPIO_IN);
  gpio_pull_up(ENCODER);
  gpio_set_irq_enabled_with_callback(ENCODER, GPIO_IRQ_EDGE_FALL, true,
                                     &encoder_callback);

  // Set up a repeating timer for motor control (10 Hz = 100 ms interval)
  int timer_0_hz = 200;
  repeating_timer_t timer_0;

  if (!add_repeating_timer_us(-1000000 / timer_0_hz, timer_0_callback, NULL,
                              &timer_0)) {
    printf("Failed to add timer\n");
    return 1;
  }

  // set up timer to update current values
  int timer_1_hz = 10000;
  repeating_timer_t timer_1;

  if (!add_repeating_timer_us(-1000000 / timer_1_hz, timer_1_callback, NULL,
                              &timer_1)) {
    printf("Failed to add timer 1\n");
    return 1;
  }

  // Align the rotor before starting the motor
  align_rotor(pwm_a, pwm_b, pwm_c);
  // Call alarm_callback in 1000 ms
  alarm_id_t alarm = add_alarm_in_ms(1000, alarm_callback, NULL, false);

  if (!alarm) {
    printf("Failed to alarm to start motor\n");
    return 1;
  }
  while (!timer_fired) {
    move_clockwise_pwm(pwm_a, pwm_b, pwm_c);
  }

  printf("Motor started\n");
  // Main loop
  while (1) {

    // update current read
    if (timer_currents_status) {
      three_phase = get_current_ab();
      quadrature = get_clark_transform(three_phase);
      rotated = get_park_transform(quadrature);
      reference_voltage = update_control(rotated);
      quadrature_voltage = get_inverse_park_transform(rotated);
      duty_cycle = get_space_vector(quadrature_voltage);
      motor_control(duty_cycle, pwm_a, pwm_b, pwm_c);

      timer_currents_status = 0;
    }

    // Encoder handling
    if (encoder_status) {
      current_angle += 1.8; // Example increment for each encoder pulse

      // Reset the angle if it exceeds 360 degrees
      if (current_angle >= 360.0) {
        current_angle -= 360.0; // Reset by subtracting 360 degrees
      }

      encoder_status = 0;
    }

    // Move the motor based on the timer callback
    // move_clockwise();
    // move_clockwise_pwm(pwm_a, pwm_b, pwm_c);
    // gpio_put(EN1, 1);
    // gpio_put(EN2, 1);
    // gpio_put(EN3, 1);
    // pwm_set_chan_level(pwm_a_slice, pwm_a_chan, 4096 * 0.083);
    // pwm_set_chan_level(pwm_b_slice, pwm_b_chan, 4096 * 0.167);
    // pwm_set_chan_level(pwm_c_slice, pwm_c_chan, 4096 * 0.25);
  }
}
