/**
 * @file mpx_evo9.h
 * @author Copyright (c) 2012 Braiins Systems s.r.o. All rights reserved.
 * @brief Multiplex Royal Evo9 driver
 */
/**
 * @defgroup group_mpx_evo9 Multiplex Royal Evo9 driver
 * @brief Multiplex Royal Evo9 namespace
 */
#ifndef _LIB_RC_MULTIPLEX_MPX_EVO9_H_
#define _LIB_RC_MULTIPLEX_MPX_EVO9_H_

/* driver framework includes */
#include <freertos-drivers/uart/uart.h>
#include <freertos-drivers/gpio/gpio_pin.h>

/* lib-rc includes */
#include <lib-rc/rc_channels.h>

/**
 * This class handles communication with Royal Evo9 transmitter via
 * UART interface.
 *
 * After initial handshake it reads servo positions for individual
 * channels and enqueues them for further processing. A dedicated task
 * is needed (see mpx_evo9__rx_task() for details) for properly
 * reading all royal evo data.
 *
 * The class currently handles protocol of the 2.6x firmware that
 * supports standard 35MHz and 40MHz RF modules. Therefore, it acts as
 * if it was one of these modules running on an arbitrary channel
 * without a scanner.
 *
 *
 * RF connector pins:
 *  1 - Tx->Rf
 *  2 - RF /EN -
 *  3 - Vbat (7.2V)
 *  4 - GND
 *  5 - Rf -> Tx
 *  6 - antenna (35 and 40MHz only, not suitable for 2.4GHz signal)
 *
 * This version of the firmware still supports standard PPM modules as
 * well as intelligent RF modules that communicate via a proprietary
 * serial protocol (see further). Therefore, we have to handle 'RF
 * enable' pin so that the UART won't get enabled until signaled via
 * this pin. That way the transmitter switches to serial mode instead
 * of PPM
 *
 * For serial mode, pin 4 (GND) has to be connected via a pulldown to
 * pin 5 (RF->Tx)
 *
 * Firmware 2.6x protocol description:

 * Initial communication uses 19200 8N2 - 2 stop bits, the description
 * @ stoeckli.net is thus imprecise. However, the setup will work with
 * 19200 8N1 since the number of stop bits doesn't matter to the UART
 * peripheral. The startup follows this procedure:
 *
 * 0) Transmitter deasserts (logic 1) RF /EN signal and waits ~100ms
 *
 * 1) Transmitter asserts (logic 0) RF /EN signal and waits
 * ~24ms. During this period, it is necessary to raise the transmit
 * line (pin 5) to idle (enable UART). The UART on the RF module
 * cannot be active prior to this otherwise the transmitter would
 * switch from UART communication to PPM and would assume a dummy
 * module that doesn't provide any feed back. Note: starting with
 * version 3.x of MPX firmware the transmitter doesn't support PPM
 * modules anymore. The following steps in the protocol describe the
 * serial protocol
 *
 * 2) Transmitter sends 'v' @ 19200 8N2 asking for RF module version
 *
 * 3) Rf module must respond with a firmware ID
 *
 * 4) Transmitter sends '?' @ 19200 8N2 asking for active frequency and
 * scanner information
 *
 * 5) Rf module sends a response that contains the currently selected
 * channel and frequency. When scanner is present, the information
 * from the scanner in binary form is provided.
 *
 * 6) Transmitter sends 'a' @ 19200 8N2 asking for enabling the Rf
 * functionality on the module and switches to 115200 8N1 baudrate
 *
 * 7) The transmitter starts sending data of the individual control
 * channels in a simple format - see stoeckli.net and related
 * documents for details.
 *
 * The data is sent @ 115200 8N1 rate in fixed frame sizes. The size
 * of each frame contains is determined by the number of control
 * position slots. Firmware 2.60 generates 12 slots, where each slot
 * is 2 bytes in size. The complete frame looks like this:
 *
 * @verbatim
 * byte: meaning
 * ---------------------------
 * 0: number of valid channels in the frame
 * 1: LSB for channel 1
 * 2: MSB for channel 1
 * 3: LSB for channel 2
 * 4: MSB for channel 2
 * .
 * .
 *
 * 23: LSB for channel 12
 * 24: MSB for channel 12
 * --------------------------
 * Total frame length: 25 bytes
 * @endverbatim
 *
 * On Royal Evo 9 bytes 19-24 are always 0 as the transmitter doesn't
 * provide any mechanism to setup the use of these channels.
 *
 * Channels, that have no controls associated (are not active) in a
 * given model setup contain 0.
 *
 * The channel positions are sent as 12 bit signed values in 1's
 * complement code in 2 byte pairs.
 *
 * +110% servo position is represented as 0x0790 (1936)
 *
 * -110% servo position is represented as 0xf86f (-1936)
 *
 * @ingroup group_mpx_evo9
 */
struct mpx_evo9 {
  /** serial interface connected to the transmitter */
  struct uart *uart;

  /** rf enable pin - transmitter signals */
  const struct gpio_pin *rf_en_gpio;

  /** queue for communicating the servo positions to the reader. It is
   * sufficient to have 1 element queue since buffering makes no sense
   * here. If the reader is not fast enough to fetch the servo
   * positions on time, something must be going wrong (e.g. broken RF
   * link) */
  xQueueHandle servo_channels_queue;

  /** statistics to keep track of how many servo position frames had
   * to be dropped since the reader didn't fetch them */
  unsigned long drop_count;
};


/**
 * Creates new instance and initializes communication with the
 * transmitter.
 *
 * @memberof mpx_evo9
 * @param uart_id - uart interface for communication with the multiplex
 * transmitter
 * @param rf_en_gpio_cfg - configuration for the GPIO connected to RF /EN pin
 *
 * @return new instance or NULL upon failure
 */
struct mpx_evo9* mpx_evo9__new(uart__id_t uart_id,
			       const struct platform_gpio_config *rf_en_gpio_cfg);


/**
 * Initializes communication with the transmitter.
 *
 * @memberof mpx_evo9
 * @param *self - this royal evo instance
 * @param uart_id - uart interface for communication with the multiplex
 * transmitter
 * @param rf_en_gpio_cfg - configuration for the GPIO connected to RF /EN pin
 *
 * @return E_OK when initialization has succeeded
 */
int mpx_evo9__init(struct mpx_evo9 *self, uart__id_t uart_id,
		   const struct platform_gpio_config *rf_en_gpio_cfg);


/**
 * This method handles incoming servo positions from the transmitter,
 * normalizes them and enqueues each frame for further processing.
 *
 * It should be called from a high priority task so that the
 * synchronization with the transmitter is never lost. Each set of
 * servo positions is being enqueued. If it doesn't fit anymore into
 * the queue it is simply dropped.
 *
 * @memberof mpx_evo9
 * @param *self - this royal evo instance
 */
void mpx_evo9__rx_task(struct mpx_evo9 *self);


/**
 * Blocking operation that waits until the transmitter provides a new
 * vector of servo channels.
 *
 * @memberof mpx_evo9
 * @param *self - this royal evo instance
 * @param *channels - output parameter for storing the received
 * channels
 */
int mpx_evo9__read(struct mpx_evo9 *self, struct rc_channels *channels);


/**
 * Resets the frame drop count.
 *
 * @memberof mpx_evo9
 * @param *self - this royal evo instance
 */
static inline void mpx_evo9__reset_drop_count(struct mpx_evo9 *self)
{
  self->drop_count = 0;
}


/**
 * Drop count accessor.
 *
 * @memberof mpx_evo9
 * @param *self - this royal evo instance
 * @return current drop count value
 */
static inline unsigned long mpx_evo9__get_drop_count(struct mpx_evo9 *self)
{
  return self->drop_count;
}

#endif /* _LIB_RC_MULTIPLEX_MPX_EVO9_H_ */
