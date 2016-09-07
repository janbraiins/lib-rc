/**
 * @file jeti_exbus.c
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI EXBUS protocol driver
 */
/* C library includes*/
#include <string.h>

#include <lib-rc/jeti/jeti_exbus.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>

/* lib RTOS extensions includes */
#include <lib-rtos/error.h>
#include <lib-rtos/byteorder/byteorder.h>

#include <lib-algorithms/crc/crc.h>

/* lib-cobjs includes */
#include <lib-cobjs/cobjs.h>

#include <lib-rc/jeti/crc16_ccit_lsb.h>

#include <lib-rc/jeti/jeti_ex.h>
#include "jeti_ex.h"

/** @addtogroup group_jeti_exbus
 *  @{
 */
/**
 * Low speed baudrate
 */
#define JETI_EXBUS__BAUD_LS 125000
/**
 * High speed baudrate
 */
#define JETI_EXBUS__BAUD_HS 250000



/** RC channels count on EXBUS */
#define JETI_EXBUS__MAX_RC_CHANNELS_COUNT 16
/** @} */


/**
 * RC channel values mapping for servo signals
 *
 * lsb is: 1/8 * 1e-3 ms
 * @ingroup group_jeti_exbus
 */
typedef enum {
  /** 1ms / lsb = 8000 */
  JETI_EXBUS__MIN_RC_CHANNEL_VALUE = 8000,
  /** 1.5ms / lsb = 12000 */
  JETI_EXBUS__CENTER_RC_CHANNEL_VALUE = 12000,
  /** 2ms / lsb = 12000 */
  JETI_EXBUS__MAX_RC_CHANNEL_VALUE = 16000,
} jeti_exbus__rc_channel_values_t;

#if (CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT < JETI_EXBUS__MAX_RC_CHANNELS_COUNT)
#error CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT too small for JETI EXBUS support
#endif

/**
 * Possible header identifiers
 * @ingroup group_jeti_exbus
 */
typedef enum {
  /** Header identifier */
  JETI_EXBUS__MASTER_HEADER_1 = 0x3e,
  /** Alternative header identifier */
  JETI_EXBUS__MASTER_HEADER_2 = 0x3d,
  /** Slave response header identifier */
  JETI_EXBUS__SLAVE_HEADER = 0x3b,
} jeti_exbus__header_id_t;


/**
 *  Packet types
 *
 * @ingroup group_jeti_exbus
 */
typedef enum {
  /** Slave response is requested */
  JETI_EXBUS__REQUEST_SLAVE_RESPONSE = 1,
  /** Slave response is not allowed */
  JETI_EXBUS__DENY_SLAVE_RESPONSE = 3,
} jeti_exbus__packet_type_t;


/**
 *  Packet types
 *
 * @ingroup group_jeti_exbus
 */
typedef enum {
  /** RC channel values data block type */
  JETI_EXBUS__RC_CHANNELS_DATA_BLOCK = 0x31,
  /** Telemetry data block request and response */
  JETI_EXBUS__TELEMETRY_DATA_BLOCK = 0x3a,
  /** JETIBox menu */
  JETI_EXBUS__JETIBOX_MENU_DATA_BLOCK = 0x3b,
} jeti_exbus__data_block_type_t;


/**
 * EXBUS packet header
 *
 * @ingroup group_jeti_exbus
 */
struct jeti_exbus__packet_header {
  /** Possible ID's see jeti_exbus__header_id_t */
  uint8_t header_id;
  /** Packet types - see jeti_exbus__packet_type_t */
  uint8_t packet_type;
  /** Packet length */
  uint8_t len;
  /** Packet ID use used as identifier for response */
  uint8_t packet_id;
} __attribute__ ((packed));

/**
 * Data block header (packet may contain multiple data blocks)
 *
 * @ingroup group_jeti_exbus
 */
struct jeti_exbus__data_block_header {
  /** Data block ID - see jeti_exbus__data_block_type_t */
  uint8_t data_id;
  /** Length of the data block */
  uint8_t len;
} __attribute__ ((packed));


/**
 * Miscellaneous private constants for JETI EXBUS
 * @ingroup group_jeti_exbus
 */
enum {
  /** Maximum theretical data block size */
  JETI_EXBUS__MAX_DATA_BLOCK_SIZE = 255 - sizeof(struct jeti_exbus__packet_header),

  /** The queue for storing servo positions is 1 element deep - this is
   * sufficient since no buffering is needed. Any servo position frames,
   * that won't fit into the queue, will get dropped. */
  JETI_EXBUS__SERVO_CHANNELS_QUEUE_LENGTH = 1,

  /** EX messages count */
  JETI_EXBUS__TELEMETRY_QUEUE_LENGTH = 4,
};


/**
 * This structure is used for overlaying a memory buffer with a potential
 * datablock that is to be created or decoded
 * @ingroup group_jeti_exbus
 */
struct jeti_exbus__data_block {
  /** Data block header */
  struct jeti_exbus__data_block_header header;
  /** Datablock payload - we use maximum size */
  uint8_t payload[255];
};


/**
 * Helper method that calculates pure packet payload size
 *
 * @memberof jeti_exbus
 * @private
 * @param self
 * @param *packet - packet being investigated
 * @return pure payload size of the packet
 */
static inline size_t
jeti_exbus__get_packet_payload_size(struct jeti_exbus *self, uint8_t *packet)
{
  struct jeti_exbus__packet_header *packet_header =
    (struct jeti_exbus__packet_header*)packet;
  return packet_header->len - sizeof(uint16_t);
}


struct jeti_exbus* jeti_exbus__new(uart__id_t uart_id)
{
  COBJS__STD_NEW_BODY(jeti_exbus, uart_id);
}


int jeti_exbus__init(struct jeti_exbus *self, uart__id_t uart_id)
{
  int retval = E_OK;

  self->servo_channels_queue = xQueueCreate(JETI_EXBUS__SERVO_CHANNELS_QUEUE_LENGTH,
                                            (unsigned portBASE_TYPE)
                                              sizeof(struct rc_channels));

  if (self->servo_channels_queue == NULL) {
    retval = E_NO_MEM;
    goto init_failed;
  }

  jeti_exbus__reset_drop_count(self);

  self->ex_telem_msg_queue = xQueueCreate(JETI_EXBUS__TELEMETRY_QUEUE_LENGTH,
                                          (unsigned portBASE_TYPE)
                                            JETI_EX__MAX_MESSAGE_SIZE);

  if (self->ex_telem_msg_queue == NULL) {
    retval = E_NO_MEM;
    goto init_failed;
  }

  self->uart = uart__new(uart_id,
                         JETI_EXBUS__BAUD_LS,
                         64,
                         portMAX_DELAY,
                         portMAX_DELAY);
  if (self->uart == NULL) {
    retval = E_FAIL;
    goto init_failed;
  }
  crc__init(&self->crc, &crc16_ccit_lsb);
  memset(&self->last_servo_positions, 0, sizeof(self->last_servo_positions));

 init_failed:
  return retval;
}


int jeti_exbus__submit_ex_msg(struct jeti_exbus *self, uint8_t *ex_msg)
{
  int retval = E_OK;

  /* Enqueue the EX telemetry message or indicate an error if the queue is
   * full */
  if(xQueueSend(self->ex_telem_msg_queue, ex_msg, 0) != pdPASS) {
    retval = E_NO_MEM;
  }

  return retval;
}


/**
 * Helper method that finalizes the response packet and transmits it to
 * the receiver.
 *
 * @memberof jeti_exbus
 * @private
 * @param self
 * @param packet_id - unique identifier of the request packet
 * @param payload_size - size of the response
 */
static void jeti_exbus__send_response(struct jeti_exbus *self,
                                      unsigned int packet_id,
                                      size_t payload_size)
{
  struct jeti_exbus__packet_header *packet_header;
  union{
    uint16_t value;
    uint8_t bytes[2];
  } crc16;
  /* Packet size without CRC16 */
  size_t packet_size;

  /* Map the packet header and fill in basic items */
  packet_header = (struct jeti_exbus__packet_header*)&self->response_packet[0];
  packet_header->header_id = JETI_EXBUS__SLAVE_HEADER;
  packet_header->packet_type = JETI_EXBUS__REQUEST_SLAVE_RESPONSE;
  packet_header->packet_id = packet_id;

  packet_header->len = sizeof(*packet_header) + payload_size + sizeof(crc16);

  assert(packet_header->len <= JETI_EXBUS__MAX_PACKET_SIZE);




  packet_size = jeti_exbus__get_packet_payload_size(self,
                                                    self->response_packet);
  /* Calculate the CRC of the packet */
  crc__reset(&self->crc);
  /* Perform CRC calculation - clamp the block size to prevent crossing
   * packet boundary */
  crc__add_block(&self->crc, self->response_packet, packet_size);

  /* Store the resulting CRC at the end of the packet */
  crc16.value = byteorder__cpu_to_le16((uint16_t)crc__get_result(&self->crc));
  self->response_packet[packet_size] = crc16.bytes[0];
  self->response_packet[packet_size + 1] = crc16.bytes[1];

  /* Send the packet and CRC */
  uart__write(self->uart, self->response_packet, packet_header->len);
}


/**
 * Update and post current servo positions values based exbus data.
 *
 * @memberof jeti_exbus
 * @private
 * @param self - this ex bus instance
 * @param packet_id - packet ID - not used, since RC channels data block
 * doesn't require our response
 * @param rc_channels_data - datablock containg the actual servo channel data
 */
static void jeti_exbus__handle_rc_channels(struct jeti_exbus *self,
                                           unsigned int packet_id,
                                           struct jeti_exbus__data_block
                                           *rc_channels_data)
{
  int i;
  /* normalized values of all servo positions that will get sent for
   * further processing */
  struct jeti_exbus__rc_channels {
    uint16_t channels[JETI_EXBUS__MAX_RC_CHANNELS_COUNT];
  } *channel_info;
  channel_info = (struct jeti_exbus__rc_channels*)rc_channels_data->payload;

  /* Convert servo positions to native signed 16-bit format */
  for (i = 0; i < JETI_EXBUS__MAX_RC_CHANNELS_COUNT; i++) {
      uint16_t channel = byteorder__le16_to_cpu(channel_info->channels[i]);

      rc_channels__set(&self->last_servo_positions, i,
                       ((int16_t)channel) -
                         (int16_t)JETI_EXBUS__CENTER_RC_CHANNEL_VALUE);
    }
  /* post the servo positions for further processing or just mark it
   * as dropped */
  if (xQueueSend(self->servo_channels_queue,
                 &self->last_servo_positions, 0) != pdPASS) {
    self->drop_count++;
  }
}


/**
 * Update and post current servo positions values based exbus data.
 *
 * @memberof jeti_exbus
 * @private
 * @param self - this ex bus instance
 * @param packet_id - packet ID - not used, since RC channels data block
 * doesn't require our response
 * @param data_block - is empty
 */
static void jeti_exbus__handle_jetibox_request(struct jeti_exbus *self,
                                                 unsigned int packet_id,
                                                 struct jeti_exbus__data_block
                                                 *data_block)
{
  struct jeti_exbus__data_block *response_data_block;
  const uint8_t *telem_data;
  size_t telem_size;
  static const uint8_t test_jetibox_menu[32] =
    "Pipe: 235mm, rpm 36000";

  telem_data = test_jetibox_menu;
  telem_size = sizeof(test_jetibox_menu);

  //response_data_block = jeti_exbus__get_response_data_block(self);
  /* Map the telemetry data block and fill in its header */
  response_data_block =
    (struct jeti_exbus__data_block*)&self->response_packet[sizeof
    (struct jeti_exbus__packet_header)];
  response_data_block->header.data_id = JETI_EXBUS__JETIBOX_MENU_DATA_BLOCK;
  response_data_block->header.len = telem_size;
  memcpy(response_data_block->payload, telem_data, telem_size);

  jeti_exbus__send_response(self, packet_id,
                            sizeof(response_data_block->header) +
                            response_data_block->header.len);
}


/**
 * EX Telemetry request handler fetches any available EX message and submits it.
 *
 *
 *
 * @param self
 * @param packet_id - reference to the telemetry request packet - it is part
 * of the response
 * @param data_block
 */
static void jeti_exbus__handle_telemetry_request(struct jeti_exbus  *self,
                                                 unsigned int packet_id,
                                                 struct jeti_exbus__data_block
                                                 *data_block)
{
  struct jeti_exbus__data_block *response_data_block;

  /* Map the response data block within the response packet */
  response_data_block =
    (struct jeti_exbus__data_block*)&self->response_packet[sizeof
      (struct jeti_exbus__packet_header)];
  response_data_block->header.data_id = JETI_EXBUS__TELEMETRY_DATA_BLOCK;

  /** Fetch EX telemetry message from the user and send it to the receiver */
  if (xQueueReceive(self->ex_telem_msg_queue, response_data_block->payload,
                    0) == pdPASS) {
    /* Adjust the response data block length based on the telemetry message
     * size */
    response_data_block->header.len =
      jeti_ex__get_msg_len(response_data_block->payload);

    jeti_exbus__send_response(self, packet_id,
                              sizeof(response_data_block->header) +
                                response_data_block->header.len);

  }
}


/**
 * Process specified data block
 *
 * @memberof jeti_exbus
 * @private
 * @param self
 * @param packet_id
 * @param data_block
 */
static void
jeti_exbus__process_data_block(struct jeti_exbus *self,
                               unsigned int packet_id,
                               struct jeti_exbus__data_block *data_block)
{
  switch (data_block->header.data_id) {
    case JETI_EXBUS__RC_CHANNELS_DATA_BLOCK:
      jeti_exbus__handle_rc_channels(self, packet_id, data_block);
      break;
    case JETI_EXBUS__TELEMETRY_DATA_BLOCK:
      jeti_exbus__handle_telemetry_request(self, packet_id, data_block);
      break;
    case JETI_EXBUS__JETIBOX_MENU_DATA_BLOCK:
      jeti_exbus__handle_jetibox_request(self, packet_id, data_block);
      break;
    default:
      assert(0);
      break;
  }
}


void jeti_exbus__rx_task(struct jeti_exbus *self)
{
  struct jeti_exbus__packet_header *packet_header;
  struct jeti_exbus__data_block_header *data_block_header;
  unsigned long crc_result;
  size_t packet_idx;
  uint16_t crc16;

  /* Map the packet header */
  packet_header = (struct jeti_exbus__packet_header*)&self->packet[0];
  memset(packet_header, 0, sizeof(*packet_header));

  /* Synchronize to the header identifier */
  while (!((packet_header->header_id == JETI_EXBUS__MASTER_HEADER_1) ||
           (packet_header->header_id == JETI_EXBUS__MASTER_HEADER_2))) {
    uart__getc(self->uart, (char*)&self->packet[0]);
  }
  /* Read the rest of the header */
  uart__read(self->uart, &self->packet[1], sizeof(*packet_header) - 1);

  /* Read all data blocks of the packet, initial index into the packet data
   * is moved after the packet header */
  packet_idx = sizeof(*packet_header);
  while(packet_idx < jeti_exbus__get_packet_payload_size(self, self->packet)) {
    /* Map the data block header */
    data_block_header =
      (struct jeti_exbus__data_block_header*)&self->packet[packet_idx];
    /* Make sure block header still fits into the packet storage buffer */
    if ((packet_idx + sizeof(*data_block_header)) > JETI_EXBUS__MAX_PACKET_SIZE)
      break;
    uart__read(self->uart, &self->packet[packet_idx], sizeof(*data_block_header));
    packet_idx += sizeof(*data_block_header);
    /* Make sure data block still fits into the packet storage buffer */
    if ((packet_idx + data_block_header->len) > JETI_EXBUS__MAX_PACKET_SIZE)
      break;
    uart__read(self->uart, &self->packet[packet_idx], data_block_header->len);
    packet_idx += sizeof(*data_block_header) + data_block_header->len;
  }
  uart__read(self->uart, &crc16, sizeof(crc16));

  // TODO consolidate this into jeti_exbus__process_packet(self, crc16);
  crc__reset(&self->crc);
  /* Perform CRC calculation - clamp the block size to prevent crossing
   * packet boundary */
  crc__add_block(&self->crc, self->packet,
                 MIN(JETI_EXBUS__MAX_PACKET_SIZE,
                     jeti_exbus__get_packet_payload_size(self, self->packet)));
  crc_result = crc__get_result(&self->crc);

  /* Process all data blocks if packet has valid checksum */
  if (crc_result == (unsigned long)byteorder__le16_to_cpu(crc16)) {
    packet_idx = sizeof(*packet_header);
    /* Iterate through all data blocks and process them */
    while (packet_idx < jeti_exbus__get_packet_payload_size(self,
                                                            self->packet)) {
      struct jeti_exbus__data_block *data_block;
      data_block = (struct jeti_exbus__data_block*)&self->packet[packet_idx];
      jeti_exbus__process_data_block(self, packet_header->packet_id,
                                     data_block);
      packet_idx += sizeof(*data_block_header) + data_block->header.len;
    }
  }
}


/**
 * @todo this operation currently blocks indefinitely, however the
 * implementation is ready for user specified timeout handling.
 */
int jeti_exbus__read(struct jeti_exbus *self, struct rc_channels *channels)
{
  int err = E_OK;
  if (xQueueReceive(self->servo_channels_queue, channels,
                    portMAX_DELAY) != pdPASS)
    err = E_TIMEOUT;

  return err;
}
