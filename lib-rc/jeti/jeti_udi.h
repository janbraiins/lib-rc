/**
 * @file jeti_udi.h
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI UDI12/16 RC channel data parser
 */
/**
 * @defgroup group_jeti_udi Jeti UDI driver
 * @brief Jeti UDI namespace
 */
#ifndef _LIB_RC_JETI_JETI_UDI_H_
#define _LIB_RC_JETI_JETI_UDI_H_

/* driver framework includes */
#include <freertos-drivers/uart/uart.h>
#include <freertos-drivers/gpio/gpio_pin.h>

/* lib-rc includes */
#include <lib-rc/rc_channels.h>

#include <task.h>

/**
 * This class handles communication with JETI receivers that UDI12/16
 * channel serial output.
 *
 *
 * The data is sent @ 115200 8N1 rate in fixed frame sizes. The size
 * of each frame contains is determined by the UDI protocol type - either
 * 12 or 16
 *
 * @verbatim
 * byte: meaning (
 * ---------------------------
 * 0: start byte (0xa1 - UDI12(12 channels), 0xa2 - UDI16 - 16 channels
 * 1: MSB for channel 1
 * 2: LSB for channel 1
 * 3: MSB for channel 2
 * 4: LSB for channel 2
 * .
 * .
 *
 * 23: MSB for channel 12
 * 24: LSB for channel 12
 * 25: CRC16 MSB -> polynom is 0x1021
 * 26: CRC16 LSB
 * --------------------------
 * Total frame length: 27 bytes (31 bytes for UDI 16)
 * @endverbatim
 *
 * The channel positions are sent as 12 bit values:
 *
 * 0x000 - min left/pull/low throttle
 * 0x120 - -110 %
 * 0x1C0 - -100 %
 * 0x800 - 0 % - Center
 * 0xE40 - +100 %
 * 0xEE0 - +110 %
 * 0xFFF - max right/push/high throttle
 *
 * @ingroup group_jeti_udi
 */
struct jeti_udi {
  /** serial interface connected to the receiver */
  struct uart *uart;

  /** queue for communicating the servo positions to the reader. It is
   * sufficient to have 1 element queue since buffering makes no sense
   * here. If the reader is not fast enough to fetch the servo
   * positions on time, something must be going wrong (e.g. broken RF
   * link) */
  QueueHandle_t servo_channels_queue;
  /* Last snapshot of servo positions */
  struct rc_channels last_servo_positions;

  /** statistics to keep track of how many servo position frames had
   * to be dropped since the reader didn't fetch them */
  unsigned long drop_count;
};


/**
 * @ingroup group_jeti_udi
 */
typedef enum {
  /** 12 RC channel mode */
  JETI_UDI__12 = 12,
  /** 16 RC channel mode */
  JETI_UDI__16 = 16,
} jeti_udi__format_t;


/**
 * Creates new instance and initializes communication with the
 * receiver.
 *
 * @memberof jeti_udi
 * @param uart_id - uart interface for communication with the jeti
 * receiver
 * @param udi_format - expected frame length see jeti_uid__format_t
 *
 * @return new instance or NULL upon failure
 */
struct jeti_udi* jeti_udi__new(uart__id_t uart_id,
                               jeti_udi__format_t udi_format);


/**
 * Initializes communication with the receiver.
 *
 * @memberof jeti_udi
 * @param *self - this jeti instance
 * @param uart_id - uart interface for communication with the jeti
 * receiver
 * @param udi_format - expected frame length see jeti_uid__format_t
 *
 * @return E_OK when initialization has succeeded
 */
int jeti_udi__init(struct jeti_udi *self, uart__id_t uart_id,
                   jeti_udi__format_t udi_format);


/**
 * This method handles incoming servo positions from the receiver,
 * normalizes them and enqueues each frame for further processing.
 *
 * It should be called from a high priority task so that the
 * synchronization with the receiver is never lost. Each set of
 * servo positions is being enqueued. If it doesn't fit anymore into
 * the queue it is simply dropped.
 *
 * @memberof jeti_udi
 * @param *self - this jeti instance
 */
void jeti_udi__rx_task(struct jeti_udi *self);


/**
 * Blocking operation that waits until the receiver provides a new
 * vector of servo channels.
 *
 * @memberof jeti_udi
 * @param *self - this jeti instance
 * @param *channels - output parameter for storing the received
 * channels
 */
int jeti_udi__read(struct jeti_udi *self, struct rc_channels *channels);


/**
 * Resets the frame drop count.
 *
 * @memberof jeti_udi
 * @param *self - this jeti instance
 */
static inline void jeti_udi__reset_drop_count(struct jeti_udi *self)
{
  self->drop_count = 0;
}


/**
 * Drop count accessor.
 *
 * @memberof jeti_udi
 * @param *self - this jeti instance
 * @return current drop count value
 */
static inline unsigned long jeti_udi__get_drop_count(struct jeti_udi *self)
{
  return self->drop_count;
}

#endif /* _LIB_RC_JETI_JETI_UDI_H_ */
