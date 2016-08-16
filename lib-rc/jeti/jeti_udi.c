/**
 * @file jeti_udi.c
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI UDI12/16 RC channel data parser
 */
/* C library includes*/
#include <string.h>

#include <lib-rc/jeti/jeti_udi.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>

/* lib RTOS extensions includes */
#include <lib-rtos/error.h>
#include <lib-rtos/byteorder/byteorder.h>

/* lib-cobjs includes */
#include <lib-cobjs/cobjs.h>

/** @addtogroup group_jeti_udi
 *  @{
 */
/**
 * Baudrate that JETI receivers use for UDI12/16 protocol frames
 */
#define JETI_UDI__BAUD 115200


#define JETI_UDI__MAX_CHANNELS JETI_UDI__16

/** The queue for storing servo positions is 1 element deep - this is
 * sufficient since no buffering is needed. Any servo position frames,
 * that won't fit into the queue, will get dropped. */
#define JETI_UDI__SERVO_CHANNELS_QUEUE_LENGTH 1
/** @} */

#if (CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT < jeti_udi__16)
#error The number of configured RC channels (CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT) is not sufficient for Royal Evo9 support!
#endif

/**
 * Channel information frame as sent by the receiver in UDI12
 *
 * @ingroup group_jeti_udi
 */
#if 0
struct jeti_udi__12_channel_info {
  union {
    uint8_t payload[25];
    struct {
      /** UDI12 uses 0xA1, UDI16 uses 0xA2 */
      uint8_t manufacturer_id;
      /* servo positions in big endian byte order
       * code */
      uint16_t channels[12];
    } s __attribute__ ((packed));
  } u;
  /** CRC channels */
  uint16_t crc16;
} __attribute__ ((packed));
#endif


struct jeti_udi__12_channel_info {
  union {
    uint8_t payload[25];
    struct {
      uint8_t manufacturer_id;
      union {
        uint8_t payload[24];
        uint16_t channels[12];
      } u __attribute__ ((packed));
    } s;
  } u1;
  uint16_t crc16;
} __attribute__ ((packed));


/**
 * Channel information frame as sent by the receiver in UDI16
 *
 * @ingroup group_jeti_udi
 */
struct jeti_udi__16_channel_info {
  /** UDI16 uses 0xA2 */
  uint8_t manufacturer_id;
  /* servo positions in big endian byte order
   * code */
  uint16_t channels[16];
  /** CRC channels */
  uint16_t crc16;
} __attribute__ ((packed));




struct jeti_udi* jeti_udi__new(uart__id_t uart_id,
                               jeti_udi__format_t udi_format)
{
  COBJS__STD_NEW_BODY(jeti_udi, uart_id, udi_format);
}


int jeti_udi__init(struct jeti_udi *self, uart__id_t uart_id,
                   jeti_udi__format_t udi_format)
{
  int retval = E_OK;

  self->servo_channels_queue = xQueueCreate(JETI_UDI__SERVO_CHANNELS_QUEUE_LENGTH,
                                            (unsigned portBASE_TYPE)
                                              sizeof(struct rc_channels));

  if (self->servo_channels_queue == NULL) {
    retval = E_NO_MEM;
    goto init_failed;
  }

  jeti_udi__reset_drop_count(self);


  self->uart = uart__new(uart_id,
                         JETI_UDI__BAUD,
                         sizeof(struct jeti_udi__12_channel_info),
                         portMAX_DELAY,
                         portMAX_DELAY);
  if (self->uart == NULL) {
    retval = E_FAIL;
    goto init_failed;
  }

 init_failed:
  return retval;
}


void jeti_udi__rx_task(struct jeti_udi *self)
{
  /* storage for the channel information in MPX native format */
  struct jeti_udi__12_channel_info channel_info;
  /* normalized values of all servo positions that will get sent for
   * further processing */
  struct rc_channels servo_positions;
  int i, j;
  uint16_t crc;

  memset(&channel_info, 0, sizeof(channel_info));
#if 0
  while (channel_info.u.s.manufacturer_id != 0xa1) {
    /*uart__getc(self->uart, (char*)&channel_info.u.s.manufacturer_id);*/
#endif
  while (channel_info.u1.s.manufacturer_id != 0xa1) {
    uart__getc(self->uart, (char*)&channel_info.u1.s.manufacturer_id);
  }
  /* read the frame sent by the receiver and verify CRC16 checksum */
  uart__read(self->uart, channel_info.u1.s.u.channels,
             sizeof(struct jeti_udi__12_channel_info) - 1);
#if 0

  uart__read(self->uart, channel_info.u.s.channels,
	     sizeof(struct jeti_udi__12_channel_info) -
         sizeof(channel_info.u.s.manufacturer_id));
#endif
  // TODO: rework to use the lib-algorithms implementation
  crc = 0xffff;
  for(i = 0; i < 25; i++) {
    crc = crc ^ channel_info.u1.payload[i] << 8;
    for(j = 0; j < 8; j++) {
      if(crc & 0x8000) {
        crc = crc << 1 ^ 0x1021;
      }
      else {
        crc = crc << 1;
      }
    }
  }
#if 0
  if (crc == byteorder__be16_to_cpu(channel_info.crc16)) {
#endif
    /* Convert servo positions to native signed 16-bit format */
    for (i = 0; i < JETI_UDI__12; i++) {
      uint16_t channel = byteorder__be16_to_cpu(channel_info.u1.s.u
                                                  .channels[i]);

#if 0
      int16_t channel = byteorder__be16_to_cpu(channel_info.u.s.channels[i]);

      rc_channels__set(&servo_positions, i, (int16_t)channel) -;
#endif
      rc_channels__set(&servo_positions, i, ((int16_t)channel) - (int16_t)
                                                                   0x800);
#if 0
    }
#endif
    /* post the servo positions for further processing or just mark it
     * as dropped */
    if (xQueueSend(self->servo_channels_queue, &servo_positions, 0) != pdPASS) {
      self->drop_count++;
    }
    /* Provide it also as the last servo positions */
    memcpy(&self->last_servo_positions, &servo_positions, sizeof(servo_positions));
  }
}


/**
 * @todo this operation currently blocks indefinitely, however the
 * implementation is ready for user specified timeout handling.
 */
int jeti_udi__read(struct jeti_udi *self, struct rc_channels *channels)
{
  int err = E_OK;
  if (xQueueReceive(self->servo_channels_queue, channels,
                    portMAX_DELAY) != pdPASS)
    err = E_TIMEOUT;

  return err;
}
