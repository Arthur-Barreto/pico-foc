#include "utils.h"

// Timer callback to control motor movement
bool timer_0_callback(repeating_timer_t *rt) {
  timer_status = 1;
  return true;
}

// timer to update currents values
bool     timer_1_callback(repeating_timer_t *rt) {
  timer_currents_status = 1;
  return true;
}

        int64_t alarm_callback(alarm_id_t id, void *user_data) {
        timer_fired = true;
        return 0;
        }

// Encoder interrupt callback
void encoder_callback(uint gpio, uint32_t events) {
  if (gpio == ENCODER) {
    encoder_status = 1;
  }
}

// Function to align rotor to a known position
void align_rotor(pwm_config_space_vector pwm_a, pwm_config_space_vector pwm_b,
                 pwm_config_space_vector pwm_c) {

  gpio_put(EN1, 1);
  gpio_put(EN2, 0);
  gpio_put(EN3, 1);
  // Set the motor to a known position
  pwm_set_chan_level(pwm_a.slice_num, pwm_a.chan_num, 1 * PWM_RES);
  pwm_set_chan_level(pwm_b.slice_num, pwm_b.chan_num, 0 * PWM_RES);
  pwm_set_chan_level(pwm_c.slice_num, pwm_c.chan_num, 0 * PWM_RES);

  current_angle = 330.0; // Assume rotor is now aligned to 330 degrees

  printf("Rotor aligned at 330 degrees.\n");

  busy_wait_ms(500); // Give time for the rotor to stabilize

  // Disable all windings after alignment
  gpio_put(EN1, 0);
  gpio_put(EN2, 0);
  gpio_put(EN3, 0);
}

void init_pwm(int pwm_pin_gp, uint resolution, uint *slice_num,
              uint *chan_num) {
  gpio_set_function(pwm_pin_gp, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(pwm_pin_gp);
  uint chan = pwm_gpio_to_channel(pwm_pin_gp);
  pwm_set_clkdiv(slice, 125); // pwm clock should now be running at 1MHz
  pwm_set_wrap(slice, resolution);
  pwm_set_chan_level(slice, PWM_CHAN_A, 0);
  pwm_set_enabled(slice, true);

  *slice_num = slice;
  *chan_num = chan;
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

void move_clockwise_pwm(pwm_config_space_vector pwm_a,
                        pwm_config_space_vector pwm_b,
                        pwm_config_space_vector pwm_c) {

  if (timer_status) {
    // Set motor pins based on the current step
    pwm_set_chan_level(pwm_a.slice_num, pwm_a.chan_num,
                       in_seq[step_index][0] * PWM_RES);
    pwm_set_chan_level(pwm_b.slice_num, pwm_b.chan_num,
                       in_seq[step_index][1] * PWM_RES);
    pwm_set_chan_level(pwm_c.slice_num, pwm_c.chan_num,
                       in_seq[step_index][2] * PWM_RES);

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

voltage_pi update_control(current_park cur_park) {
  voltage_pi res;
  id_error = id_ref - cur_park.cur_d;
  iq_error = iq_ref - cur_park.cur_q;

  id_integrator += id_error;
  iq_integrator += iq_error;

  res.v_d = kp * id_error + ki * id_integrator;
  res.v_q = kp * iq_error + ki * iq_integrator;

  // clamp id integrator
  if (id_integrator > integrator_max) {
    id_integrator = integrator_max;
  } else if (id_integrator < -integrator_max) {
    id_integrator = -integrator_max;
  }

  // clamp iq integrator
  if (iq_integrator > integrator_max) {
    iq_integrator = integrator_max;
  } else if (iq_integrator < -integrator_max) {
    iq_integrator = -integrator_max;
  }

  return res;
}

voltage_clark get_inverse_park_transform(current_park cur_park) {
  voltage_clark res;
  res.v_alpha =
      cos(current_angle) * cur_park.cur_d - sin(current_angle) * cur_park.cur_q;
  res.v_beta =
      sin(current_angle) * cur_park.cur_d + cos(current_angle) * cur_park.cur_q;
  return res;
}

space_vector get_space_vector(voltage_clark cur_clark) {
  space_vector res;





  // Step 1: Calculate V_ref and theta_ref (in radians)
  float     v_ref = sqrt(cur_clark.v_alpha * cur_clark.v_alpha +
                     cur_clark.v_beta * cur_clark.v_beta);
  float theta_ref = atan2(cur_clark.v_beta, cur_clark.v_alpha);

  // Step 2: Determine the sector (radians divided by pi/3, which is 60 degrees)
    uint8_t sector = (uint8_t)(floor(theta_ref / (M_PI / 3))) + 1;
         if (sector > 6) {
    sector = 6; // Sector must be between 1 and 6
  }

  // Step 3: Calculate switching times T1, T2, T0 based on the sector
  float ts = 1.0 / PWM_FREQ; // PWM period (1 MHz frequency as you specified)
  float angle_in_sector =
      theta_ref - (sector - 1) * (M_PI / 3); // Relative angle within sector

  float t1 = ts * (v_ref * sin((M_PI / 3) - angle_in_sector)) /
             MOTOR_VOLTAGE; // Time for first active vector
  float t2 = ts * (v_ref * sin(angle_in_sector)) /
             MOTOR_VOLTAGE;  // Time for second active vector
  float t0 = ts - (t1 + t2); // Zero vector time (remaining time)

  // Step 4: Calculate duty cycles for phases A, B, C
  float duty_a, duty_b, duty_c;
  switch (sector) {
  case 1:
    duty_a = (t1 + t2 + t0 / 2) / ts;
    duty_b = (t2 + t0 / 2) / ts;
    duty_c = t0 / (2 * ts);
    break;
  case 2:
    duty_a = (t2 + t0 / 2) / ts;
    duty_b = (t1 + t2 + t0 / 2) / ts;
    duty_c = t0 / (2 * ts);
    break;
  case 3:
    duty_a = t0 / (2 * ts);
    duty_b = (t1 + t2 + t0 / 2) / ts;
    duty_c = (t2 + t0 / 2) / ts;
    break;
  case 4:
    duty_a = t0 / (2 * ts);
    duty_b = t0 / (2 * ts);
    duty_c = (t1 + t2 + t0 / 2) / ts;
    break;
  case 5:
    duty_a = (t2 + t0 / 2) / ts;
    duty_b = t0 / (2 * ts);
    duty_c = (t1 + t2 + t0 / 2) / ts;
    break;
  case 6:
    duty_a = (t1 + t2 + t0 / 2) / ts;
    duty_b = t0 / (2 * ts);
    duty_c = (t2 + t0 / 2) / ts;
    break;
  default:
    duty_a = 0;
    duty_b = 0;
    duty_c = 0;
    break;
  }






  // Step 5: Store the results in the structure
  res.duty_a = duty_a;
  res.duty_b = duty_b;
  res.duty_c = duty_c;

  return res;
}

void     motor_control(space_vector duty_cycle, pwm_config_space_vector pwm_a,
                   pwm_config_space_vector pwm_b,
                   pwm_config_space_vector pwm_c) {
  pwm_set_chan_level(pwm_a.slice_num, pwm_a.chan_num,
                     (uint16_t)(duty_cycle.duty_a * PWM_RES));
        pwm_set_chan_level(pwm_b.slice_num, pwm_b.chan_num,
        (uint16_t)(duty_cycle.duty_b * PWM_RES));
  pwm_set_chan_level(pwm_c.slice_num, pwm_c.chan_num,
                     (uint16_t)(duty_cycle.duty_c * PWM_RES));

                gpio_put(EN1, duty_cycle.duty_a == 0 ? 0 : 1);
    gpio_put(EN2, duty_cycle.duty_b == 0 ? 0 : 1);
        gpio_put(EN3, duty_cycle.duty_c == 0 ? 0 : 1);
}
