/**
 * @file rclink.cpp
 * @brief Full duplex XBee RC link between transmitter and receiver
 */
#include <lib-rc/rflink/rflink.hpp>

#include <lib-rtos/time.h>

/* XBee wrapper*/
#include "rclinkxbeewrapper.h"

#include <string.h>

/* default queue lengths  */
#define TX_STATUS_QUEUE_LENGTH 16
#define RX_PAYLOAD_QUEUE_LENGTH 8
#define AT_CMD_STATUS_QUEUE_LENGTH 16


/**
 * Access for the RCLinkXBeeWrapper instance
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 *
 * @return pointer to the RCLinkXBeeWrapper instance
 */
static inline RCLinkXBeeWrapper* rflink__get_xbee_wrapper(struct rflink *self)
{
  return (RCLinkXBeeWrapper*)(self->xbee);
}


/**
 * Locks the associated XBee device
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 */
static inline void rflink__lock_xbee(struct rflink *self)
{
  xSemaphoreTake(self->xbee_lock, portMAX_DELAY);
}


/**
 * Unlocks the associated XBee device
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 */
static inline void rflink__unlock_xbee(struct rflink *self)
{
  xSemaphoreGive(self->xbee_lock);
}


/**
 * Sends an AT command with an optional data payload
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 * @param at_command - pointer to the AT command being sent (2 bytes expected)
 * @param at_command_data - data associated with the AT command
 * @param at_command_data_length - length of the data for the AT
 * command
 */
static void rflink__send_at_command(struct rflink *self,
				    const uint8_t at_command[],
				    const uint8_t at_command_data[],
				    uint8_t at_command_data_length)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);

  rflink__lock_xbee(self);
  xbee_wrapper->submitAtCommandRequest(at_command, at_command_data,
				       at_command_data_length);
  rflink__unlock_xbee(self);
}


/**
 * Waits for an AT command response.
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 * @param *response - stores the resulting response
 * @param expected_at_cmd - the AT command that is expected in the
 * response
 * @param timeout - maximum time to wait for the response in OS
 * ticks. portMAX_DELAY waits infinitely.
 *
 * @return RFLINK__SUCCESS - if a proper response has arrived and
 * indicates no error
 */
static int rflink__recv_at_command_response(struct rflink *self,
					    struct at_command_response *response,
					    const uint8_t expected_at_cmd[],
					    portTickType timeout)
{
  int err = RFLINK__SUCCESS;

  /* fetch the AT response */
  if (xQueueReceive(self->at_command_response_queue, response,
		    timeout) != pdPASS) {
    err = RFLINK__AT_COMMAND_RESPONSE_TIMEOUT;
    goto receive_failed;
  }
  /* ensure that the received response is for the desired command */
  if (response->command[0] != expected_at_cmd[0] ||
      response->command[1] != expected_at_cmd[1]) {
    err = RFLINK__UNEXPECTED_AT_COMMAND;
    goto receive_failed;
  }
  if (response->status != AT_OK) {
    err = RFLINK__AT_COMMAND_FAILED;
    goto receive_failed;
  }

 receive_failed:
  return err;
}


/**
 * Common part of the link initialization
 *
 * The initialization performs network reset, so that a new channel is
 * selected (in case of coordinator) or the router attaches to the new
 * network
 *
 * @private
 * @memberof rflink
 * @param *self - this RC link instance
 *
 * @return RFLINK__SUCCESS if it has been successfully initialized
 */
static void rflink__reset_xbee(self)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);


  rflink__lock_xbee(self);
  /* assert reset for 10 ms */
  gpio_pin__wr(self->reset_gpio, 0);
  vTaskDelay(TIME__MS_TO_OS_TICKS(choose_some_delay);
  gpio_pin__wr(self->reset_gpio, 1);
  xbee_wrapper->getXBee().
  xbee_wrapper->submitZBTxRequest(buf, len);

  /* reset drop count statistics */
  self->rx_payload_drop_count = 0;
  self->tx_status_drop_count = 0;
  self->at_command_response_drop_count = 0;

  rflink__unlock_xbee(self);

  xbee_lock

  while (self->xbee_lock
}


/**
 * \<\<private\>\> Common part of the link initialization
 *
 * The initialization performs network reset, so that a new channel is
 * selected (in case of coordinator) or the router attaches to the new
 * network
 *
 * @param *self - this RC link instance
 *
 * @return RFLINK__SUCCESS if it has been successfully initialized
 */
static int rflink__init(struct rflink *self)
{
  int err = RFLINK__SUCCESS;
  struct at_command_response response;
  uint8_t at_cmd[] = {'N', 'R'};

  rflink__reset_xbee(self);

  rflink__send_at_command(self, at_cmd, NULL, 0);
  rflink__handle_rx_frame(self);
  err = rflink__recv_at_command_response(self, &response, at_cmd,
					NETWORK_RESET_TIMEOUT);

  if (err != RFLINK__SUCCESS)
    goto init_failed;

  rflink__handle_rx_frame(self);
  if (self->modem_status != DISASSOCIATED) {
    err = RFLINK__UNEXPECTED_MODEM_STATUS;
    goto init_failed;
  }

  rflink__handle_rx_frame(self);
  if (self->modem_status != COORDINATOR_STARTED) {
    err = RFLINK__UNEXPECTED_MODEM_STATUS;
    goto init_failed;
  }

 init_failed:
  return err;
}


/**
 * \<\<private\>\> Receives the Rx response.
 *
 * This method fetches the response from the relevant queue and
 * provides a copy of it into user provided buffer
 *
 * @param *self - this RC link instance
 * @param *response - output parameter for storing the received Rx response
 * @param timeout - maximum time to wait for the response in OS
 * ticks. portMAX_DELAY waits infinitely.

 */
static inline int rflink__recv_zb_rx_response(struct rflink *self,
					     struct zb_rx_response *response,
					     portTickType timeout)
{
  int err = RFLINK__SUCCESS;
  /* fetch the Rx response */
  if (xQueueReceive(self->rx_payload_queue, response,
		    timeout) != pdPASS)
    err = RFLINK__RX_TIMEOUT;

  return err;
}

/**
 * \<\<private\>\> Handles the AT command response.
 *
 * The method copies all relevant data out of AT command response and
 * posts it onto the queue for further processing.
 *
 * Responses that won't fit into the queue are dropped and this is
 * indicated by incrementing the relevant drop counter.
 *
 * @param *self - this RC link instance
 */
static void rflink__handle_rx_at_command_response(struct rflink *self)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);
  AtCommandResponse &at_command_response =
    xbee_wrapper->getAtCommandResponse();
  /* response that will be sent down the queue */
  struct rflink__at_command_response response;
  size_t data_length;

  response.frame_id = at_command_response.getFrameId();
  memcpy(response.command, at_command_response.getCommand(), 2);
  response.status = at_command_response.getStatus();

  data_length = at_command_response.getValueLength();
  /* flag the payload as too large and truncate the data length if
   * necessary */
  if (data_length > AT_COMMAND_RESPONSE_MAX_DATA_LENGTH) {
    response.status |= AT_COMMAND_RESPONSE_PAYLOAD_TOO_LARGE;
    data_length = AT_COMMAND_RESPONSE_MAX_DATA_LENGTH;
  }
  if (data_length != 0)
    memcpy(response.data, at_command_response.getValue(), data_length);

  response.data_length = (uint8_t)data_length;

  /* post the response for further processing or just mark it as
   * dropped */
  if (xQueueSend(self->at_command_response_queue, &response, 0) != pdPASS)
    self->at_command_response_drop_count++;
}


/**
 *\<\<private\>\> Handles the ZigBee Rx response.
 *
 * This method copies all relevant data out of the Rx response and
 * posts it onto the queue for further processing.
 *
 * Responses that won't fit into the queue are dropped and this is
 * indicated by incrementing the relevant drop counter.
 *
 * @param *self - this RC link instance
 */
static void rflink__handle_rx_zb_rx_response(struct rflink *self)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);
  ZBRxResponse &zb_rx_response =
    xbee_wrapper->getZBRxResponse();
  /* response that will be sent down the queue */
  struct zb_rx_response response;
  size_t data_length;

  response.address64_msb = zb_rx_response.getRemoteAddress64().getMsb();
  response.address64_lsb = zb_rx_response.getRemoteAddress64().getLsb();
  response.address16 = zb_rx_response.getRemoteAddress16();
  response.receive_options = zb_rx_response.getOption();

  data_length = zb_rx_response.getDataLength();
  /* flag the payload as too large and truncate the data length if
   * necessary */
  if (data_length > CONFIG_LIBRFLINK__RX_MAX_PAYLOAD) {
    response.receive_options |= ZB_RX_RESPONSE_PAYLOAD_TOO_LARGE;
    data_length = CONFIG_LIBRFLINK__RX_MAX_PAYLOAD;
  }
  if (data_length != 0)
    memcpy(response.data, zb_rx_response.getFrameData() +
	   zb_rx_response.getDataOffset(), data_length);

  response.data_length = (uint8_t)data_length;

  /* post the response for further processing or just mark it as
   * dropped */
  if (xQueueSend(self->rx_payload_queue, &response, 0) != pdPASS)
    self->rx_payload_drop_count++;
}


/**
 * \<\<private\>\> Handles ZigBee Tx status response.
 *
 *
 * The delivery status is posted onto the relevant queue.
 *
 * @param *self - this RC link instance
 */
static void rflink__handle_rx_zb_tx_status_response(struct rflink *self)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);
  ZBTxStatusResponse &zb_tx_status_response =
    xbee_wrapper->getZBTxStatusResponse();
  uint8_t delivery_status = zb_tx_status_response.getDeliveryStatus();

  /* post the status for further processing or just mark it as
   * dropped */
  if (xQueueSend(self->tx_status_queue, &delivery_status, 0) != pdPASS)
    self->tx_status_drop_count++;
}


 struct rclink* rflink__new(struct uart *xbee_uart,
			   const struct platform_gpio_config *rf_en_gpio_cfg)
{
  struct rflink *self;
  RCLinkXBeeWrapper *xbee_wrapper;

  self = (struct rclink*)pvPortMalloc(sizeof(struct rclink));

  if (self == NULL)
    goto fail;

  /* create the XBee wrapping object */
  xbee_wrapper = new RCLinkXBeeWrapper();
  if (xbee_wrapper == NULL)
    goto xbee_wrapper_failed;

  self->xbee = (void*)xbee_wrapper;
  xbee_wrapper->getXBee().setSerial(xbee_uart);

  self->xbee_lock = xSemaphoreCreateMutex();

  /* provide all queues */
  self->rx_payload_queue = xQueueCreate(RX_PAYLOAD_QUEUE_LENGTH,
					(unsigned portBASE_TYPE)
					sizeof(struct zb_rx_response));

  self->tx_status_queue = xQueueCreate(TX_STATUS_QUEUE_LENGTH,
				       (unsigned portBASE_TYPE)
				       sizeof(uint8_t));

  self->at_command_response_queue = xQueueCreate(AT_CMD_STATUS_QUEUE_LENGTH,
						 (unsigned portBASE_TYPE)
						 sizeof(struct at_command_response));

  gpio_port_initXXXXX(reset_gpio_config);
  rflink__reset_xbee(self);

  return self;

  /* error handling */
 xbee_wrapper_failed:
  vPortFree(self);
 fail:
  return NULL;
}


int rflink__init_transmitter(struct rflink *self)
{
  int err;
  struct zb_rx_response response;
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);
  /* map the payload onto the receiver ID message */
  struct rflink__receiver_id *rx_id = (struct rflink__receiver_id*)response.data;

  /* generic initialization */
  if ((err = rflink__init(self)) != RFLINK__SUCCESS)
    goto init_failed;

  /* wait for the connecting router ID frame */
  rflink__handle_rx_frame(self);

  /* fetch the Rx response and detect the receiver frame. */
  rflink__recv_zb_rx_response(self, &response, INIT_TIMEOUT);
  if (response.data_length != sizeof(struct rflink__receiver_id)) {
    err = RFLINK__TRANSMITTER_INIT_FAILED;
    goto init_failed;
  }

  if (rx_id->id != RX_ID) {
    err = RFLINK__TRANSMITTER_INIT_FAILED;
    goto init_failed;
  }

  /* receiver has been identified properly, we store its address */
  xbee_wrapper->setZBTxDestAddress64(response.address64_msb,
				     response.address64_lsb);

 init_failed:
  return err;
}


int rflink__get_rssi(struct rflink *self, int *rssi)
{
  int err;
  uint8_t at_cmd[] = {'D', 'B'};
  struct at_command_response response;

  rflink__send_at_command(self, at_cmd, NULL, 0);
  err = rflink__recv_at_command_response(self, &response, at_cmd,
					AT_COMMAND_TIMEOUT);

  if (err == RFLINK__SUCCESS) {
    *rssi = -1 * (int)response.data[0];
  }
  return err;
}


int rflink___get_channel(struct rflink *self, int *channel)
{
  int err;
  uint8_t at_cmd[] = {'C', 'H'};
  struct at_command_response response;

  rflink__send_at_command(self, at_cmd, NULL, 0);
  err = rflink__recv_at_command_response(self, &response, at_cmd,
					AT_COMMAND_TIMEOUT);

  if (err == RFLINK__SUCCESS) {
    *channel = (int)response.data[0];
  }
  return err;
}


void rflink___handle_rx_frame(struct rflink *self)
{
  RCLinkXBeeWrapper *xbee_wrapper = rflink___get_xbee_wrapper(self);
  XBee &xbee = xbee_wrapper->getXBee();

  xbee.readPacket();

  switch (xbee.getResponse().getApiId()) {
  case AT_COMMAND_RESPONSE:
    rflink__handle_rx_at_command_response(self);
    break;
  case ZB_TX_STATUS_RESPONSE:
    rflink__handle_rx_zb_tx_status_response(self);
    break;
  case ZB_RX_RESPONSE:
    rflink__handle_rx_zb_rx_response(self);
    break;
  case MODEM_STATUS_RESPONSE:
    /* there is no queuing of the modem status, we can handle it
     * directly here */
    self->modem_status = xbee_wrapper->getModemStatusResponse().getStatus();
    break;
  default:
    break;
  }
}


int rflink__recv(struct rflink *self, void *buf,
		 portTickType timeout)
{
  int err;

  err = rflink__recv_zb_rx_response(self, (struct zb_rx_response*)buf,
				   timeout);
  return err;
}


int rflink__send(struct rflink *self, const void *buf, size_t len,
		 portTickType timeout, uint8_t *delivery_status)
{
  int err = RFLINK__SUCCESS;
  RCLinkXBeeWrapper *xbee_wrapper = rflink__get_xbee_wrapper(self);
  uint8_t ret_delivery_status;

  rflink__lock_xbee(self);
  xbee_wrapper->submitZBTxRequest(buf, len);
  rflink__unlock_xbee(self);

  /* fetch the Tx status */
  if (xQueueReceive(self->tx_status_queue, &ret_delivery_status,
		    timeout) != pdPASS)
    err = RFLINK__TX_TIMEOUT;


  if (delivery_status != NULL)
    *delivery_status = ret_delivery_status;

  /* anything else than SUCCESS delivery status (see XBee.h - TX
   * STATUS constants) indicates transmission failure */
  if (ret_delivery_status != SUCCESS)
    err = RFLINK__TX_DELIVERY_FAILED;

  return err;
}
