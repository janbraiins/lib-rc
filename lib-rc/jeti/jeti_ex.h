/**
 * @file jeti_ex.h
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI EX telemetry driver
 */
/**
 * @defgroup group_jeti_ex JETI EX Telemetry driver group
 * @brief JETI EX Telemetry driver namespace
 */
#ifndef _LIB_RC_JETI_JETI_EX_H_
#define _LIB_RC_JETI_JETI_EX_H_

#include <lib-algorithms/crc/crc.h>

/* lib-rc includes */
#include <lib-rc/jeti/jeti_ex_sensor.h>


/**
 * This class represents a set of sensors for EX Telemetry protocol.
 * The class is capable to serialize the sensor values and labels into
 * EX messages.
 *
 * @ingroup group_jeti_ex
 */
struct jeti_ex {
  /** A list of sensors to be managed */
  struct jeti_ex_sensor *sensors;
  /** Number of sensors in the set - the protocol limits the number of
   * sensors to 15! */
  unsigned int sensor_count;

  /** Index of the next sensor value to be reported - EX message size is
   * limitted to JETI_EX__MAX_MESSAGE_SIZE. Therefore reporting all sensors
   * in a set requires fragmenting the values among multiple messages */
  unsigned int next_sensor_value_idx;
  /** Index of the next sensor description to be reported */
  unsigned int next_sensor_descr_idx;

  /** Manufacturer ID of the sensor set */
  uint16_t manufacturer_id;
  /** Device ID of the sensor set */
  uint16_t device_id;
  /** CRC calculator (initialized with CRC8-CCIT descriptor) */
  struct crc crc;
};


/**
 * The following are data type ID's as listed in the specification
 * @ingroup group_jeti_ex
 */
typedef enum {
  /* int6_t  Data type 6b (-31, 31) */
  JETI_EX__TYPE_6b = 0,
  /* int14_t Data type 14b (-8191, 8191) */
  JETI_EX__TYPE_14b = 1,
  /* int22_t Data type 22b (-2097151, 2097151) */
  JETI_EX__TYPE_22b = 4,
  /* int22_t Special data type – time and date */
  JETI_EX__TYPE_DT = 5,
  /* int30_t Data type 30b (-536870911, 536870911) */
  JETI_EX__TYPE_30b = 8,
  /* int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree. */
  JETI_EX__TYPE_GPS = 9,
} jeti_ex__data_type_t;


/**
 * Miscellaneous constants
 * @ingroup group_jeti_ex
 */
enum {
  /** Maximum possible telemetry message size as given by the protocol
   * specification including the start byte */
  JETI_EX__MAX_MESSAGE_SIZE = 29,
};


/**
 * Creates new instance and initializes communication with the
 * receiver.
 *
 * @memberof jeti_ex
 * @param sensors - see jeti_ex::sensors
 * @param sensor_count - see jeti_ex::sensor_count
 * @param manufacturer_id - see jeti_ex::manufacturer_id
 * @param device_id - see jeti_ex::device_id
 * @return new instance or NULL upon failure
 */
struct jeti_ex* jeti_ex__new(struct jeti_ex_sensor *sensors,
                             size_t sensor_count, uint16_t manufacturer_id,
                             uint16_t device_id);


/**
 * Initializes communication with the receiver.
 *
 * @memberof jeti_ex
 * @param *self - this jeti EX instance
 * @param sensors - see jeti_ex::sensors
 * @param sensor_count - see jeti_ex::sensor_count
 * @param manufacturer_id - see jeti_ex::manufacturer_id
 * @param device_id - see jeti_ex::device_id
 *
 * @return E_OK when initialization has succeeded
 */
int jeti_ex__init(struct jeti_ex *self, struct jeti_ex_sensor *sensors,
                  size_t sensor_count, uint16_t manufacturer_id,
                  uint16_t device_id);

/**
 * Creates a data message from all sensors within this jeti_ex instance
 *
 * @meberof jeti_ex
 * @param self - this jeti EX instance
 * @param msg_buf - storage for the message
 * @param msg_buf_len - maximum size of the message
 * @param actual_msg_len - actual message size
 * @return E_NO_MEM when message buffer was not big enough to fit all sensor
 * serialized data, E_OK upon success
 */
int jeti_ex__create_data_message(struct jeti_ex *self, uint8_t *msg_buf,
                                 size_t msg_buf_len,
                                 size_t *actual_msg_len);


/**
 * Creates a text message with all sensor descriptions
 *
 * @meberof jeti_ex
 * @param self - this jeti EX instance
 * @param msg_buf - storage for the message
 * @param msg_buf_len - maximum size of the message
 * @param actual_msg_len - actual message size
 * @return E_NO_MEM when message buffer was not big enough to fit
 * descriptions for all sensors, E_OK upon success
 */
int jeti_ex__create_text_message(struct jeti_ex *self, uint8_t *msg_buf,
                                 size_t msg_buf_len,
                                 size_t *actual_msg_len);


/**
 * Calculates physical size of the EX message
 *
 * @meberof jeti_ex
 * @param ex_msg - pointer to a valid EX message
 * @return The packet size consists of the message payload length (present in
 * the header) and the size of the 2 preceding items that the protocol
 * doesn't count into the length field.
 */
size_t jeti_ex__get_msg_len(uint8_t *ex_msg);


#endif /* _LIB_RC_JETI_JETI_EX_H_ */
