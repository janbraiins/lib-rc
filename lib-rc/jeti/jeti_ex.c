/**
 * @file jeti_ex.c
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI EX protocol driver
 */
/* C library includes*/
#include <string.h>

#include <lib-rc/jeti/jeti_ex.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>

/* lib RTOS extensions includes */
#include <lib-rtos/error.h>
#include <lib-rtos/byteorder/byteorder.h>
#include <lib-rtos/assert.h>

#include <lib-algorithms/crc/crc.h>

/* lib-cobjs includes */
#include <lib-cobjs/cobjs.h>

#include <lib-rc/jeti/crc8_ccit.h>


/**
 * Data length mapping of individual data types
 * @ingroup group_jeti_ex
 */
static const uint8_t jeti_ex__data_type_len[] = {
  [JETI_EX__TYPE_6b]  = 1,
  [JETI_EX__TYPE_14b] = 2,
  [JETI_EX__TYPE_22b] = 3,
  [JETI_EX__TYPE_DT]  = 3,
  [JETI_EX__TYPE_30b] = 4,
  [JETI_EX__TYPE_GPS] = 4
};

/**
 * Creates a bit mask for setting decimals
 * @ingroup group_jeti_ex
 */
#define JETI_EX_DECIMAL_MASK(sensor) (sensor->decimal_point << 5)

/**
 * @ingroup group_jeti_ex message (packet) types
 */
typedef enum {
  /** Telemetry message with text description of the values (labels/units) */
  JETI_EX__TEXT_MSG = 0,
  /** Telemetry message with actual sensor values */
  JETI_EX__DATA_MSG = 1,
} jeti_ex__msg_type_t;


/**
 * EX message header
 * @ingroup group_jeti_ex
 */
struct jeti_ex__message_header {
  /** Message identifier (anything containing 0x*F) */
  uint8_t id;
  /** Type/length byte */
  union {
    struct {
      uint8_t length:6;
      uint8_t type:2;
    } s;
    uint8_t value;
  } u_tl;
  /** Manufacturer ID (0xA400 – 0xA41F range allowed by spec) */
  uint16_t manufacturer_id;
  /** Unique device ID within the manufacturer ID */
  uint16_t device_id;

  uint8_t reserved;
} __attribute__ ((packed));


/**
 * Miscellaneous internal constants
 * @ingroup group_jeti_ex
 */
enum {
  /** Distinct identification of an EX message (0xNf), where N is arbitrary as
   * documented in the official protocol specification */
  JETI_EX__MESSAGE_START_BYTE = 0x9f,
  /** Message payload size accounts for the header */
  JETI_EX__MAX_PAYLOAD_SIZE = JETI_EX__MAX_MESSAGE_SIZE -
                              sizeof(struct jeti_ex__message_header),
  /** Maximum number of sensors that can be addressed within 1 set */
  JETI_EX__MAX_SENSOR_COUNT = 15,
};


/**
 * EX Telemetry message.
 * The payload contains the maximum possible
 * @ingroup group_jeti_ex
 */
struct jeti_ex__message {
  struct jeti_ex__message_header header;
  /** Payload with data or text descriptions of sensor values */
  uint8_t payload[JETI_EX__MAX_PAYLOAD_SIZE];
} __attribute__ ((packed));


/**
 * @ingroup group_jeti_ex
 * @todo Add some compile time check, the bitfields are for little-endian!
 */
union jeti_ex__serialized_value_header {
  struct {
    /** Data type */
    uint8_t type:4;
    /** Data identifier (0-15) */
    uint8_t id:4;
  } s;
  uint8_t id_type;
} __attribute__ ((packed));


/**
 * Represents 1 sensor value serialized in the message
 *
 * The actual size of the struct is not important as the struct is only being
 * used as an overlay for the message buffer. The size is given by the data
 * type.
 * @ingroup group_jeti_ex
 */
struct jeti_ex__serialized_value {
  /** Data header */
  union jeti_ex__serialized_value_header header;
  /** Upto 4 bytes of data */
  uint8_t data[4];
} __attribute__ ((packed));


/**
 * Header of the sensor labeling text within EX message
 * @ingroup group_jeti_ex
 */
struct jeti_ex__serialized_text_header {
  /** Identifier of the telemetry value */
  uint8_t id;
  /** Length of the telemetry sensor label and unit */
  union {
    struct {
      /** Length of unit description */
      uint8_t unit_len:3;
      /** Length of description */
      uint8_t label_len:5;
    } s;
    uint8_t unit_label_len;
  } u;
} __attribute__ ((packed));


/**
 * Represents 1 sensor labeling serialized in the message
 *
 * The actual size of the struct is not important as the struct is only being
 * used as an overlay for the message buffer. The size is given by label_len
 * and unit_len attributes.
 * @ingroup group_jeti_ex
 */
struct jeti_ex__serialized_text {
  /** Header of the serialized text */
  struct jeti_ex__serialized_text_header header;
  /** Storage for both descriptions */
  uint8_t text[JETI_EX__MAX_PAYLOAD_SIZE - sizeof(struct
    jeti_ex__serialized_text_header)];
} __attribute__ ((packed));


/**
 * Serializes a specified sensor value
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param sensor_idx
 * @param msg_buf
 */
static void jeti_ex__serialize_value(struct jeti_ex *self,
                                     unsigned int sensor_idx,
                                     uint8_t *msg_buf)
{
  unsigned int i;
  /* Mapping of the sensor data to msg buffer */
  struct jeti_ex__serialized_value *data =
    (struct jeti_ex__serialized_value*)msg_buf;
  struct jeti_ex_sensor *sensor;
  /* Temporary storage for the little endian converted sensor value */
  union {
    int32_t value;
    uint8_t bytes[4];
  } u;

  /** Sanity check for sensor index. */
  assert((sensor_idx > 0) && (sensor_idx < self->sensor_count));
  sensor = &self->sensors[sensor_idx];

  data->header.s.id = (uint8_t)sensor_idx;
  data->header.s.type = sensor->descr->data_type;

  u.value = byteorder__cpu_to_le32(sensor->value);

  /* Store sensor data - without the last byte */
  for (i = 0; i < jeti_ex__data_type_len[sensor->descr->data_type] - 1; i++) {
    data->data[i] = u.bytes[i];
  }
  /* Last data byte contains extra flags */
  data->data[i] = u.bytes[i] | sensor->descr->decimal_point_mask;
  /* Sign bit detection */
  if (sensor->value < 0)
    data->data[i] |= 0x80;
}

/**
 * Calculates physical size of the EX packet
 *
 * @param ex_msg - pointer to a valid EX packet
 * @return The packet size consists of the message payload length (present in
 * the header) and the size of the 2 preceding items that the protocol
 * doesn't count into the length field.
 */
size_t jeti_ex__get_msg_len(uint8_t *ex_msg)
{
  struct jeti_ex__message_header *ex_msg_header =
    (struct jeti_ex__message_header*)ex_msg;

  return sizeof(ex_msg_header->id) + sizeof(ex_msg_header->u_tl) +
         ex_msg_header->u_tl.s.length;
}
/**
 * Serializes text description of the specified sensor
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param sensor_idx
 * @param msg_buf
 */
static void jeti_ex__serialize_text(struct jeti_ex *self,
                                    unsigned int sensor_idx,
                                    uint8_t *msg_buf)
{
  /* Mapping of the sensor data to msg buffer */
  struct jeti_ex__serialized_text *text =
    (struct jeti_ex__serialized_text*)msg_buf;
  const struct jeti_ex_sensor__descr *descr;

  assert(sensor_idx < self->sensor_count);
  descr = self->sensors[sensor_idx].descr;
  text->header.id = sensor_idx;
  text->header.u.s.unit_len = descr->unit_len;
  text->header.u.s.label_len = descr->label_len;

  memcpy(text->text, descr->label, descr->label_len);
  memcpy(text->text + descr->label_len, descr->unit,
         descr->unit_len);
}


/**
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param sensor_idx
 * @return Expected size of the sensor label and unit descriptions when
 * serialized into the EX message including the header
 */
static size_t jeti_ex__get_serialized_text_size(struct jeti_ex *self,
                                                unsigned int sensor_idx)
{
  assert(sensor_idx < self->sensor_count);
  const struct jeti_ex_sensor__descr *descr = self->sensors[sensor_idx].descr;
  return sizeof(struct jeti_ex__serialized_text_header) +
         descr->label_len + descr->unit_len;

}


/**
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param sensor_idx
 * @return Expected size of the sensor value when serialized into the EX
 * message including the header (identifier and data type)
 */
static size_t jeti_ex__get_serialized_value_size(struct jeti_ex *self,
                                                 unsigned int sensor_idx)
{
  assert(sensor_idx < self->sensor_count);
  const struct jeti_ex_sensor__descr *descr = self->sensors[sensor_idx].descr;

  return sizeof(union jeti_ex__serialized_value_header) +
         jeti_ex__data_type_len[descr->data_type];
}


/**
 * Helper method for filling in the EX message header
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param header
 * @param msg_type
 * @param msg_length
 */
static inline void
jeti_ex__set_ex_header(struct jeti_ex *self,
                       struct jeti_ex__message_header *header,
                       jeti_ex__msg_type_t msg_type,
                       size_t msg_length)
{
  header->id = JETI_EX__MESSAGE_START_BYTE;
  header->manufacturer_id = self->manufacturer_id;
  header->device_id = self->device_id;
  header->u_tl.s.type = msg_type;
  /* EX message length is exotic - it doesn't contain the ID and type/length
   * fields*/
  header->u_tl.s.length = sizeof(*header) - (sizeof(header->id) +
                                             sizeof(header->u_tl)) +
                          msg_length + /* CRC8 */1;
}


/**
 * Helper method that creates an arbitrary EX message method
 *
 * @memberof jeti_ex
 * @private
 * @param self
 * @param msg_type - type of the message to be created
 * @param msg_buf - storage buffer for the message
 * @param msg_buf_len - maximum message size
 * @param get_serialized_size - method for determining serialized size of a
 * sensor (one of jeti_ex__get_serialized_value_size() or
 * jeti_ex__get_serialized_text_size())
 * @param serialize - method for serializing sensor into ex message (one of
 * jeti_ex__serialize_value() or jeti_ex__serialize_text())
 * @param actual_msg_len - output argument - size of the created message
 *
 * @return E_NO_MEM when message buffer is not big enough, E_OK when message
 * has been succesfully created
 */
int jeti_ex__create_message(struct jeti_ex *self, jeti_ex__msg_type_t msg_type,
                            uint8_t *msg_buf,
                            size_t msg_buf_len,
                            unsigned int *next_sensor_idx,
                            unsigned int sensor_msg_limit,
                            size_t (*get_serialized_size)(
                              struct jeti_ex*, unsigned int),
                            void (*serialize)(struct jeti_ex*, unsigned int,
                                              uint8_t*),
                            size_t *actual_msg_len)
{
  int retval = E_OK;
  unsigned int sensors_processed = 0;
  size_t msg_buf_space_left;
  size_t payload_idx = 0;
  struct jeti_ex__message *msg = (struct jeti_ex__message*)msg_buf;
  /* At least message header and CRC has to fit into the provided message
   * buffer*/
  if (msg_buf_len < (sizeof(msg->header) + 1)) {
    retval = E_NO_MEM;
    goto no_mem;
  }
  /* Allocate space for the header in the buffer */
  msg_buf_space_left = msg_buf_len - sizeof(msg->header);

  /* Scan all sensors starting at the last unprocessed sensor when this
   * method was called */
  while((msg_buf_space_left > 1) &&
        (*next_sensor_idx < self->sensor_count) &&
        (sensors_processed < sensor_msg_limit)) {
    size_t serialized_len = get_serialized_size(self, *next_sensor_idx);

    /* Quit upon potential overflow (+1 accounts for the CRC space) */
    if ((serialized_len + 1) > msg_buf_space_left) {
      break;
    }
    serialize(self, *next_sensor_idx, &msg->payload[payload_idx]);
    payload_idx += serialized_len;
    msg_buf_space_left -= serialized_len;
    *next_sensor_idx += 1;
    sensors_processed++;
  }
  /* Handle next sensor index overflow */
  if (*next_sensor_idx >= self->sensor_count) {
    *next_sensor_idx = 0;
  }
  /* We should always have space for CRC since the loop has been
   * terminated early enough */
  /* All sensors have been serialized, we can now fill in the header */
  jeti_ex__set_ex_header(self, &msg->header, msg_type, payload_idx);

  /* Calculate the CRC of the EX message - including the header - the
   * specification states that CRC doesn't contain the message identifier. */
  crc__reset(&self->crc);
  crc__add_block(&self->crc, &msg->header.u_tl.value,
                 sizeof(msg->header) + payload_idx - sizeof(msg->header.id));
  msg->payload[payload_idx] = (uint8_t)crc__get_result(&self->crc);


  if (actual_msg_len != NULL) {
    *actual_msg_len = jeti_ex__get_msg_len(msg_buf);
  }
  no_mem:
  return retval;

}


int jeti_ex__create_data_message(struct jeti_ex *self, uint8_t *msg_buf,
                                 size_t msg_buf_len,
                                 size_t *actual_msg_len)
{
  /* Never create message for sensor 0, since it contains only description of
   * the entire sensor */
  if (self->next_sensor_value_idx == 0) {
    self->next_sensor_value_idx++;
  }

  return jeti_ex__create_message(self, JETI_EX__DATA_MSG, msg_buf,
                                 msg_buf_len,
                                 &self->next_sensor_value_idx,
                                 JETI_EX__MAX_SENSOR_COUNT,
                                 jeti_ex__get_serialized_value_size,
                                 jeti_ex__serialize_value,
                                 actual_msg_len);
}


int jeti_ex__create_text_message(struct jeti_ex *self, uint8_t *msg_buf,
                                 size_t msg_buf_len,
                                 size_t *actual_msg_len)
{
  return jeti_ex__create_message(self, JETI_EX__TEXT_MSG, msg_buf,
                                 msg_buf_len,
                                 &self->next_sensor_descr_idx,
                                 1, /* Only 1 text description in the message */
                                 jeti_ex__get_serialized_text_size,
                                 jeti_ex__serialize_text,
                                 actual_msg_len);
}


struct jeti_ex* jeti_ex__new(struct jeti_ex_sensor *sensors,
                             size_t sensor_count, uint16_t manufacturer_id,
                             uint16_t device_id)
{
  COBJS__STD_NEW_BODY(jeti_ex, sensors, sensor_count, manufacturer_id,
                      device_id);
}


int jeti_ex__init(struct jeti_ex *self, struct jeti_ex_sensor *sensors,
                  size_t sensor_count, uint16_t manufacturer_id,
                  uint16_t device_id)
{
  int retval = E_OK;

  if (sensor_count > JETI_EX__MAX_SENSOR_COUNT) {
    retval = E_INVAL;
    goto init_failed;
  }

  self->sensors = sensors;
  self->sensor_count = sensor_count;
  self->manufacturer_id = manufacturer_id;
  self->device_id = device_id;

  self->next_sensor_value_idx = 0;
  self->next_sensor_descr_idx = 0;

  crc__init(&self->crc, &crc8_ccit);

 init_failed:
  return retval;
}

