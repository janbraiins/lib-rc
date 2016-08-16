/**
 * @defgroup group_servo Servo UDI driver
 * @brief Servo driver namespace
 */
#ifndef _LIB_RC_SERVO_SERVO_H_
#define _LIB_RC_SERVO_SERVO_H_

/* driver framework includes */
#include <freertos-drivers/pwm/pwm.h>


/**
 * This class allows driving a servo connected to a specified PWM channel.
 * It uses a simple linear mapping of user specified min/max range to valid
 * servo control signals. The resulting mapping linear function for the mapping:
 *
 * duty_cycle_ticks = (slope_numerator/slope_denominator) *
 * requested_position + offset
 *
 * Individual attributes are calculated during servo initialization based on
 * min_pulse and max_pulse parameters
 *
 * @ingroup group_servo
 */
struct servo {
  /** PWM instance that represents this servo channel */
  struct pwm pwm;
	/** Clock speed of the underlying timer of the PWM is needed for
	 * calculating the correct duty cycle and period for the servo control
	 * signal */
  unsigned long base_clock_hz;
	/** slope (numerator part) for the linear function to map the specified
	 * min/max value
   * range onto the signal pulses required by the servo, we keep it as
   * a separate 2 element for integer based calculation */
  int slope_numerator;
	/** see slope_numerator */
  int slope_denominator;
	/** offset int ticks is calculated for SERVO__MAX_PULSE_US (converted to
	 * ticks) and the requested max_pulse position (see server__new()) */
  int offset;
};


/**
 * Servo pulse limits
 *
 * @ingroup group_servo
 */
typedef enum {
  SERVO__MIN_PULSE_US = 1000,
  SERVO__CENTER_PULSE_US = 1500,
  SERVO__MAX_PULSE_US = 2000,
  SERVO__PERIOD_US = 20000,
} servo__ctrl_signal_t;


/**
 * Creates new servo instance.
 *
 * @memberof servo
 * @param pwm_id - identifies the PWM channel driving this servo
 * @param pwm_config - platform specific configuration of the channel - see
 * pwm__init()
 * @param base_clock_hz - see servo::base_clock_hz
 * @param min_pulse - value that represents the minimum servo position
 * (results in generating control pulse with duty cycle of SERVO__MIN_PULSE_US)
 * @param max_pulse - arbitrary value that represents the maximum servo
 * position (results in generating control pulse with duty cycle of
 * SERVO__MAX_PULSE_US)
 * @return servo instance or NULL upon failure
 */
struct servo* servo__new(pwm__id_t pwm_id,
												 const struct platform_pwm_config *pwm_config,
												 unsigned long base_clock_hz,
												 int min_pulse, int max_pulse);


/**
 * Servo initializer.
 *
 * @memberof servo
 * @param self - this servo instance
 * @param pwm_id - see server__new()
 * @param pwm_config - see server__new()
 * @param base_clock_hz - see server__new()
 * @param min_pulse - see server__new()
 * @param max_pulse - see server__new()
 * @return E_OK if initialization has succeeded
 */
int servo__init(struct servo *self, pwm__id_t pwm_id,
								const struct platform_pwm_config *pwm_config,
								unsigned long base_clock_hz,
								int min_pulse, int max_pulse);


/**
 * Sets servo position based on
 *
 * @memberof servo
 * @param self
 * @param channel_position
 */
void servo__set_position(struct servo *self, int channel_position);


#endif /* _LIB_RC_SERVO_SERVO_H_ */
