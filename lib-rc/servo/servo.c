/**
 * @file servo.h
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief Servo driver implementation
 */
#include <lib-rc/servo/servo.h>

/* lib-rtos includes */
#include <lib-rtos/error.h>
#include <lib-rtos/time.h>

/* lib-cobjs includes */
#include <lib-cobjs/cobjs.h>


struct servo* servo__new(pwm__id_t pwm_id,
			 const struct platform_pwm_config *pwm_config,
			 unsigned long base_clock_hz,
			 int min_pulse, int max_pulse)
{
  COBJS__STD_NEW_BODY(servo, pwm_id, pwm_config, base_clock_hz,
		      min_pulse, max_pulse);
}


int servo__init(struct servo *self, pwm__id_t pwm_id,
		const struct platform_pwm_config *pwm_config,
		unsigned long base_clock_hz,
		int min_pulse, int max_pulse)
{
  int retval;
  self->base_clock_hz = base_clock_hz;

  /* slope for the linear function to map the specified min/max pulse
   * range onto the signal pulses required by the servo, we keep it as
   * a separate 2 element for integer based calculation */
  self->slope_numerator =
    TIME__US_TO_TICKS(SERVO__MAX_PULSE_US, base_clock_hz) -
    TIME__US_TO_TICKS(SERVO__MIN_PULSE_US, base_clock_hz);
  self->slope_denominator = max_pulse - min_pulse;

  self->offset = TIME__US_TO_TICKS(SERVO__MAX_PULSE_US, base_clock_hz) -
                 ((self->slope_numerator * max_pulse) / self->slope_denominator);

  retval = pwm__init(&self->pwm, pwm_id, pwm_config);

  if (retval != E_OK) {
    goto init_failed;
  }

  pwm__set_period(&self->pwm, TIME__US_TO_TICKS(SERVO__PERIOD_US,
                                                base_clock_hz));

  pwm__set_duty(&self->pwm, TIME__US_TO_TICKS(SERVO__CENTER_PULSE_US,
                                              base_clock_hz));

  pwm__enable(&self->pwm);
 init_failed:
  return retval;
}


void servo__set_position(struct servo *self, int channel_position)
{
  unsigned long ticks;

  ticks =
    (self->slope_numerator * channel_position) / self->slope_denominator +
    self->offset;

  if (ticks > 5250)
    ticks = 5250;
  else if (ticks < 2625)
    ticks = 2625;

  pwm__set_duty(&self->pwm, ticks);
}
