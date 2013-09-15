/**
 * @file mpx_evo9.c
 * @author Copyright (c) 2012 Braiins Systems s.r.o. All rights reserved.
 * @brief Multiplex Royal Evo9 driver
 */
#include <lib-rc/multiplex/mpx_evo9.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>

/* lib RTOS extensions includes */
#include <lib-rtos/error.h>
#include <lib-rtos/byteorder/byteorder.h>

/** @addtogroup group_mpx_evo9
 *  @{
 */
/**
 * Initial baudrate used for communication that the transmitter uses
 * for communication with the RF module
 */
#define MPX_EVO9__INIT_BAUD 19200

/**
 * Final baudrate that the transmitter uses for sending servo position
 * frames
 */
#define MPX_EVO9__RF_BAUD 115200

/** The transmitter sends this number of channels (servo positions) to
 * the RF module in 1 frame */
#define MPX_EVO9__MAX_CHANNELS 12

/** The queue for storing servo positions is 1 element deep - this is
 * sufficient since no buffering is needed. Any servo position frames,
 * that won't fit into the queue, will get dropped. */
#define MPX_EVO9__SERVO_CHANNELS_QUEUE_LENGTH 1
/** @} */

#if (CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT < MPX_EVO9__MAX_CHANNELS)
#error The number of configured RC channels (CONFIG_LIB_RC_RC_CHANNELS_MAX_COUNT) is not sufficient for Royal Evo9 support!
#endif

/**
 * Channel information frame as sent by the transmitter
 *
 * @ingroup group_mpx_evo9
 */
struct mpx_evo9__channel_info {
  /** number of valid channels in the frame - see mpx_evo9 class for
   * details */
  uint8_t channel_count;
  /* servo positions in little endian byte order in 1's complement
   * code */
  uint16_t channels[MPX_EVO9__MAX_CHANNELS];
} __attribute__ ((packed));


/**
 * Performs the handshake with the transmitter - valid for firmware 2.6x
 *
 * The handshake responds to the version, scanner and enable commands
 * sent by the transmitter. The responses fake a 35MHz module using
 * channel 255.
 *
 * @memberof mpx_evo9
 * @private
 * @param *self - this royal evo instance
 *
 * @return E_OK upon success
 */
static int mpx_evo9__do_rf_handshake(struct mpx_evo9 *self)
{
  static const char fw_ver[] = "35ABHFMS2V268 Jul 11 200812:52:40\r\n";
  /* scanner response - 35MHz @ channel 255 (35.495 MHz), the meaning
   * of the scanner binary data following the frequency in ASCII has
   * not been reverse engineered */
  static const char scanner_resp[] = {'3', '5','A', 'B', '2', '5', '5', '3',
				      '4', '9', '5', '0', 0x01, 0x40, 0x00,
				      0x30, '\r', '\n'};
  int done = 0;
  int retval = E_OK;

  /* Keep processing commands from the transmitter until the 'a'
   * command is received */
  while (!done) {
    char cmd;
    ssize_t status;

    uart__read(self->uart, &cmd, sizeof(char));

    switch (cmd) {
      /* firmware version request by the transmitter */
    case 'v':
      status = uart__write(self->uart, fw_ver, sizeof(fw_ver));
      break;
      /* scanner request from the transmitter */
    case '?':
      status = uart__write(self->uart, scanner_resp, sizeof(scanner_resp));
      break;
      /* finally, the transmitter informs the module that servo
       * positions will now be transmitted @ 115.2kBaud */
    case 'a':
      uart__set_baud(self->uart, MPX_EVO9__RF_BAUD);
      done = 1;
      break;
    }
    /* check for any failure during communication with the transmitter */
    if (status < 0) {
      done = 1;
      retval = (int)status;
    }
  }

  return retval;
}


struct mpx_evo9* mpx_evo9__new(uart__id_t uart_id,
			       const struct platform_gpio_config *rf_en_gpio_cfg)
{
  struct mpx_evo9 *self;

  self = pvPortMalloc(sizeof(struct mpx_evo9));

  if (self == NULL)
    goto fail;

  if (mpx_evo9__init(self, uart_id, rf_en_gpio_cfg) != E_OK)
    goto init_failed;

  return self;

  /* error handling */
 init_failed:
  vPortFree(self);
 fail:
  return NULL;
}


int mpx_evo9__init(struct mpx_evo9 *self, uart__id_t uart_id,
		   const struct platform_gpio_config *rf_en_gpio_cfg)
{
  int retval;
  /* used when waiting for the transmitter signaling the RF module via
   * RF /EN pin. */
  portTickType last_execution_time;

  self->servo_channels_queue = xQueueCreate(MPX_EVO9__SERVO_CHANNELS_QUEUE_LENGTH,
					    (unsigned portBASE_TYPE)
					    sizeof(struct rc_channels));

  if (self->servo_channels_queue == NULL) {
    retval = E_NO_MEM;
    goto init_failed;
  }

  /* enable the pin connected to RF /EN */
  self->rf_en_gpio = gpio_pin__new(rf_en_gpio_cfg);


  if (self->rf_en_gpio == NULL) {
    retval = E_FAIL;
    goto init_failed;
  }
  mpx_evo9__reset_drop_count(self);

  last_execution_time = xTaskGetTickCount();
  /* we maybe faster to boot than the transmitter, therefore we wait
   * until the transmitter initializes (deasserts the RF /EN pin) */
  while (gpio_pin__rd(self->rf_en_gpio) == 0) {
    /* Wait until it is time for the next cycle. */
    vTaskDelayUntil(&last_execution_time, 5 / portTICK_RATE_MS);
  }

  /* sample the RF enable pin every 5ms check for enable signal (logic
   * 0 indicates enable from the transmitter) */
  while (gpio_pin__rd(self->rf_en_gpio) == 1) {
    /* Wait until it is time for the next cycle. */
    vTaskDelayUntil(&last_execution_time, 5 / portTICK_RATE_MS);
  }

  /* the transmitter has signalled RF enable, we can now enable the
   * UART that pulls the RF->Tx pin high when idle */
  self->uart = uart__new(uart_id,
			 MPX_EVO9__INIT_BAUD,
			 sizeof(struct mpx_evo9__channel_info),
			 portMAX_DELAY,
			 portMAX_DELAY);
  if (self->uart == NULL) {
    retval = E_FAIL;
    goto init_failed;
  }

  retval = mpx_evo9__do_rf_handshake(self);
 init_failed:
  return retval;
}


void mpx_evo9__rx_task(struct mpx_evo9 *self)
{
  /* storage for the channel information in MPX native format */
  struct mpx_evo9__channel_info channel_info;
  /* normalized values of all servo positions that will get sent for
   * further processing */
  struct rc_channels servo_positions;
  int i;

  /* read the frame sent by the transmitter and convert the servo
   * positions from 1's complement to 2's complement signed values */
  uart__read(self->uart, &channel_info,
	     sizeof(struct mpx_evo9__channel_info));

  assert((channel_info.channel_count >= 0) &&
	 (channel_info.channel_count <= MPX_EVO9__MAX_CHANNELS));

  for (i = 0; i < MPX_EVO9__MAX_CHANNELS; i++) {
    uint16_t channel = byteorder__le16_to_cpu(channel_info.channels[i]);
    /* detect negative value and perform the conversion to 2's complement */
    if ((channel & 0x8000) != 0)
      channel += 1;
    rc_channels__set(&servo_positions, i, (int16_t)channel);
  }

  /* post the servo positions for further processing or just mark it
   * as dropped */
  if (xQueueSend(self->servo_channels_queue, &servo_positions, 0) != pdPASS)
    self->drop_count++;

}


/**
 * @todo this operation currently blocks indefinitely, however the
 * implementation is ready for user specified timeout handling.
 */
int mpx_evo9__read(struct mpx_evo9 *self, struct rc_channels *channels)
{
  int err = E_OK;
  if (xQueueReceive(self->servo_channels_queue, channels,
		    portMAX_DELAY) != pdPASS)
    err = E_TIMEOUT;

  return err;
}
