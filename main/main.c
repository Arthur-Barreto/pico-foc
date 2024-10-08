#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define IN1 2
#define IN2 3
#define IN3 4
#define EN1 5
#define EN2 6
#define EN3 7
#define ENCODER 16
#define CUR_A 26
#define CUR_B 27

volatile int timer_status = 0;
volatile uint8_t encoder_status = 0;
volatile uint8_t step_index = 0;

float current_angle = 0.0;
float current_phase_a = 0.0;
float current_phase_b = 0.0;

uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                        {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                        {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};

const float conversion_factor = 3.3f / (1 << 12);

// Timer callback to control motor movement
bool timer_0_callback(repeating_timer_t *rt) {
  timer_status = 1;
  return true; // Keep repeating
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

  current_angle = 30.0; // Assume rotor is now aligned to 90 degrees

  printf("Rotor aligned at 30 degrees.\n");

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

float get_current_from_channel(uint8_t channel) {
  // pass 0 for channel A and 1 for channel B

  adc_select_input(channel);
  float result = adc_read();
  result = result * conversion_factor;
  return result;
}

int main() {
  stdio_init_all();

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

  // Main loop
  while (1) {
    // Move the motor based on the timer callback
    move_clockwise();

    // Encoder handling
    if (encoder_status) {
      current_angle += 1.8; // Example increment for each encoder pulse

      // Reset the angle if it exceeds 360 degrees
      if (current_angle >= 360.0) {
        current_angle -= 360.0; // Reset by subtracting 360 degrees
      }

      encoder_status = 0;
      current_phase_a = get_current_from_channel(0);
      current_phase_b = get_current_from_channel(1);
      printf("Current angle: %.2f degrees\n", current_angle);
      printf("Current phase A current: %.2f mA\n", current_phase_a);
      printf("Current phase B current: %.2f mA\n", current_phase_b);
    }
  }
}
