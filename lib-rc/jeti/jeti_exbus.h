/**
 * @file jeti_exbus.h
 * @author Copyright (c) 2016 Braiins Systems s.r.o. All rights reserved.
 * @brief JETI EXBUS driver
 */
/**
 * @defgroup group_jeti_exbus JETI EXBUS driver group
 * @brief JETI EXBUS driver namespace
 */
#ifndef _LIB_RC_JETI_JETI_EXBUS_H_
#define _LIB_RC_JETI_JETI_EXBUS_H_

/* driver framework includes */
#include <freertos-drivers/uart/uart.h>
#include <freertos-drivers/gpio/gpio_pin.h>

/* lib-rc includes */
#include <lib-rc/rc_channels.h>
#include <lib-algorithms/crc/crc.h>

#include <task.h>

/**
 * Miscellaneous constants of the EXBUS protocol
 * @ingroup group_jeti_exbus
 */
enum {
  /** Maximum supported packet size */
  JETI_EXBUS__MAX_PACKET_SIZE = 64
};

/**
 * This class handles communication with JETI receivers via EXBus protocol
 * (specification available @ http://jeti.com)
 *
 *
 * The data is sent @ 125 kBaud or 250 kBaud 8N1 rate in variable length frame
 * sizes.
 *
 *
 * @ingroup group_jeti_exbus
 */
struct jeti_exbus {
  /** serial interface connected to the receiver */
  struct uart *uart;

  /** Queue for communicating the servo positions to the reader. It is
   * sufficient to have 1 element queue since buffering makes no sense
   * here. If the reader is not fast enough to fetch the servo
   * positions on time, something must be going wrong
   */
  QueueHandle_t servo_channels_queue;
  /* Last snapshot of servo positions */
  struct rc_channels last_servo_positions;

  /** Statistics to keep track of how many servo position frames had
   * to be dropped since the reader didn't fetch them */
  unsigned long drop_count;

  /** Queue for submission of EX Telemetry messages towards the RC receiver */
  QueueHandle_t ex_telem_msg_queue;

  /** CRC calculator (initialized with CRC16-CCIT-LSB descriptor) */
  struct crc crc;

  /** Storage for the current packet that is being received/processed */
  uint8_t packet[JETI_EXBUS__MAX_PACKET_SIZE];

  /** Storage for the packet that is being sent as response */
  uint8_t response_packet[JETI_EXBUS__MAX_PACKET_SIZE];
};


/**
 * Creates new instance and initializes communication with the
 * receiver.
 *
 * @memberof jeti_exbus
 * @param uart_id - uart interface for communication with the jeti
 * receiver
 * @return new instance or NULL upon failure
 */
struct jeti_exbus* jeti_exbus__new(uart__id_t uart_id);


/**
 * Initializes communication with the receiver.
 *
 * @memberof jeti_exbus
 * @param *self - this jeti instance
 * @param uart_id - uart interface for communication with the jeti
 * receiver
 *
 * @return E_OK when initialization has succeeded
 */
int jeti_exbus__init(struct jeti_exbus *self, uart__id_t uart_id);


/**
 * Submits EX message into the queue for further transmission back to RC
 * controller.
 *
 * @param self - this EX bus instance
 * @param ex_msg - EX message to be submitted for transmission via receiver
 * @return E_NO_MEM when EX message queue is full
 */
int jeti_exbus__submit_ex_msg(struct jeti_exbus *self, uint8_t *ex_msg);


/**
 * This method handles incoming servo positions from the receiver,
 * normalizes them and enqueues each frame for further processing.
 *
 * It should be called from a high priority task so that the
 * synchronization with the receiver is never lost. Each set of
 * servo positions is being enqueued. If it doesn't fit anymore into
 * the queue it is simply dropped.
 *
 * @memberof jeti_exbus
 * @param *self - this jeti instance
 */
void jeti_exbus__rx_task(struct jeti_exbus *self);


/**
 * Blocking operation that waits until the receiver provides a new
 * vector of servo channels.
 *
 * @memberof jeti_exbus
 * @param *self - this jeti instance
 * @param *channels - output parameter for storing the received
 * channels
 */
int jeti_exbus__read(struct jeti_exbus *self, struct rc_channels *channels);


/**
 * Resets the frame drop count.
 *
 * @memberof jeti_exbus
 * @param *self - this jeti instance
 */
static inline void jeti_exbus__reset_drop_count(struct jeti_exbus *self)
{
  self->drop_count = 0;
}


/**
 * Drop count accessor.
 *
 * @memberof jeti_exbus
 * @param *self - this jeti instance
 * @return current drop count value
 */
static inline unsigned long jeti_exbus__get_drop_count(struct jeti_exbus *self)
{
  return self->drop_count;
}

#endif /* _LIB_RC_JETI_JETI_EXBUS_H_ */
