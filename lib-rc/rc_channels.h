/**
 * @file rc_channels.h
 * @author Copyright (c) 2012 Braiins Systems s.r.o. All rights reserved.
 * @brief Describes servo positions for a set of rc channels
 */
/**
 * @defgroup group_rc_channels Servo positions
 * @brief RC channels namespace
 */
#ifndef _LIB_RC_RC_CHANNELS_H_
#define _LIB_RC_RC_CHANNELS_H_

/* C-library includes */
#include <stdint.h>

#include <lib-rtos/assert.h>

/**
 * Describes servo positions for a set of rc channels.
 *
 * Each servo position is a signed int16 value. There is a library
 * configurable limit on the maximum number of rc channels
 *
 * @ingroup group_rc_channels
 */
struct rc_channels {
  /* servo positions for all channels */
  int16_t channels[CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT];
};


/**
 * Servo position setter
 *
 * @memberof rc_channels
 * @param *self - this rc_channels instance
 * @param idx - index of the servo channel to be extracted
 * @param value - servo position for a given channel
 */
static inline void rc_channels__set(struct rc_channels *self, int idx,
				    int16_t value)
{
  assert(idx < CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT);
  self->channels[idx] = value;
}

/**
 * Servo position getter
 *
 * @memberof rc_channels
 * @param *self - this rc_channels instance
 * @param idx - index of the servo channel to be extracted
 */
static inline int16_t rc_channels__get(struct rc_channels *self, int idx)
{
  assert(idx < CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT);
  return self->channels[idx];
}




#endif /* _LIB_RC_RC_CHANNELS_H_ */
