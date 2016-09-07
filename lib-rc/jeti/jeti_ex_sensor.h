/**
 * @file jeti_ex_sensor.h
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI EX telemetry driver
 */
#ifndef _LIB_RC_JETI_JETI_EX_SENSOR_H_
#define _LIB_RC_JETI_JETI_EX_SENSOR_H_

#include <stdint.h>

/**
 * Ex sensor descriptor
 *
 * @ingroup group_jeti_ex
 */
struct jeti_ex_sensor__descr {
  const char *label;
  /** Length of the label string */
  size_t label_len;
  const char *unit;
  /** Length of the unit string */
  size_t unit_len;
  /** See jeti_ex__data_type_t*/
  uint8_t data_type;
  /** Decimal point mask */
  uint8_t decimal_point_mask;
};


/**
 * @ingroup group_jeti_ex
 */
struct jeti_ex_sensor {
  /** Value of the sensor */
  int32_t value;
  /** Descriptor of the sensor */
  const struct jeti_ex_sensor__descr *descr;
};


/**
 * Static initializer
 *
 * @see jeti_ex_sensor__descr for meaning if individual parameters
 *
 * @ingroup group_jeti_ex
 * @param label_str
 * @param unit_str
 * @param type
 * @param decimal_point - supported values are: 0, 1, 2
 */
#define JETI_EX_SENSOR__DESCR_INIT(label_str, unit_str, type, decimal_point) \
  {								    \
    .label = label_str,						    \
    .label_len = sizeof(label_str) - 1,	         		    \
    .unit = unit_str,						    \
    .unit_len = sizeof(unit_str) - 1,     			    \
    .data_type = type,	         				    \
    .decimal_point_mask = decimal_point << 5, \
  }


/**
 * Creates new instance and initializes communication with the
 * receiver.
 *
 * @memberof jeti_ex_sensor
 * @param descr - sensor descriptor
 * @return new instance or NULL upon failure
 */
struct jeti_ex_sensor*
jeti_ex_sensor__new(const struct jeti_ex_sensor__descr *descr);


/**
 * Initializes communication with the receiver.
 *
 * @memberof jeti_ex_sensor
 * @param *self - this sensor instance
 * @param descr - sensor descriptor
 *
 * @return E_OK when initialization has succeeded
 */
int jeti_ex_sensor__init(struct jeti_ex_sensor *self,
                         const struct jeti_ex_sensor__descr *descr);

#endif /* _LIB_RC_JETI_JETI_EX_SENSOR_H_ */
