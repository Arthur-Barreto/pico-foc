#include "utils.h"

current_ab three_phase;
current_clark quadrature;
current_park rotated;
voltage_pi reference_voltage;
voltage_clark quadrature_voltage;

int main() {
  stdio_init_all();
  sleep_ms(2000);

  // Configure the GPIOs for the motor
  gpio_init(IN1);
  gpio_init(IN2);
  gpio_init(IN3);
  gpio_init(EN1);
  gpio_init(EN2);
  gpio_init(EN3);

  gpio_set_dir(IN1, GPIO_OUT);
  gpio_set_dir(IN2, GPIO_OUT);
  gpio_set_dir(IN3, GPIO_OUT);
  gpio_set_dir(EN1, GPIO_OUT);
  gpio_set_dir(EN2, GPIO_OUT);
  gpio_set_dir(EN3, GPIO_OUT);

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

  // Align the rotor before starting the motor
  align_rotor();

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

  // Main loop
  while (1) {

    // update current read
    if (timer_currents_status) {
      three_phase = get_current_ab();
      quadrature = get_clark_transform(three_phase);
      rotated = get_park_transform(quadrature);
      reference_voltage = update_control(rotated);
      quadrature_voltage = get_inverse_park_transform(rotated);

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
    move_clockwise();
  }
}
