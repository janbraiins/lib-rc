/**
 * @file rflink.h
 * @brief Full duplex XBee radio link between transmitter and receiver.
 */
/**
 * @defgroup group_rflink Full duplex XBee RF link
 * @brief XBee RF link namespace
 */
#ifndef _LIB_RC_RFLINK_RFLINK_H_
#define _LIB_RC_RFLINK_RFLINK_H_

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

/* freertos drivers includes*/
#include <time.h>

#include <stdint.h>

/**
 * @ingroup group_rflink
 */
typedef enum {
  RFLINK__SUCCESS = 0,
  RFLINK__UNEXPECTED_MODEM_STATUS,
  RFLINK__NETWORK_RESET_FAILED,
  RFLINK__AT_COMMAND_RESPONSE_TIMEOUT,
  RFLINK__UNEXPECTED_AT_COMMAND,
  RFLINK__AT_COMMAND_FAILED,
  RFLINK__RX_TIMEOUT,
  RFLINK__TX_TIMEOUT,
  RFLINK__TX_DELIVERY_FAILED,
  RFLINK__TRANSMITTER_INIT_FAILED,
} rflink_err_t;


/** @addtogroup group_rflink
 *  @{
 */
/** the size of the response data didn't fit into the frame */
#define RFLINK__AT_COMMAND_RESPONSE_PAYLOAD_TOO_LARGE 0x80
/** maximum size   */
#define RFLINK__AT_COMMAND_RESPONSE_MAX_DATA_LENGTH 64

/** user the is required to provide buffers of this size */
#define RFLINK__MIN_RX_BUF_SIZE (sizeof(struct zb_rx_response))

/** the size of the response data didn't fit into the frame */
#define RFLINK__ZB_RX_RESPONSE_PAYLOAD_TOO_LARGE 0x80

/** maximum time to wait for the network reset to complete */
#define RFLINK__NETWORK_RESET_TIMEOUT TIME_MS_TO_OS_TICKS(5000)
/** maximum time to wait for link initialization */
#define RFLINK__INIT_TIMEOUT TIME_MS_TO_OS_TICKS(5000)
/** maximum time to wait for AT command to complete */
#define RFLINK__AT_COMMAND_TIMEOUT TIME_MS_TO_OS_TICKS(100)

/** Magic number that identifies the receiver */
#define RFLINK__RX_ID 0xdeadbeef
/** @} */

/**
 * Describes an initialized RF-link between transmitter and receiver.
 */
struct rflink {
  /** all xbee related objects */
  void *xbee;
  /** ensure mutual access to the XBee device */
  xSemaphoreHandle xbee_lock;

  /** queue for received frames */
  xQueueHandle rx_payload_queue;
  /** queue for transmit status */
  xQueueHandle tx_status_queue;
  /** queue for AT command status */
  xQueueHandle at_command_response_queue;

  /** current modem status */
  uint8_t modem_status;
  /** number of rx payloads that have been dropped */
  uint32_t rx_payload_drop_count;
  /** number of tx status frames that have been dropped */
  uint32_t tx_status_drop_count;
  /** number of AT command responses that have been dropped */
  uint32_t at_command_response_drop_count;
};


/**
 * Describes AT command response
 */
struct rflink__at_command_response {
  /** frame ID that identifies the original AT command request frame */
  uint8_t frame_id;
  /** AT command associated with this response */
  uint8_t command[2];
  /**
   * status - see XBee.h for possible AT command statuses
   *
   * In addition, the status AT_COMMAND_RESPONSE_PAYLOAD_TOO_LARGE
   * denotes that the data payload didn't fit into this response
   * structure
   */
  uint8_t status;
  /** physical copy of the response data */
  uint8_t data[AT_COMMAND_RESPONSE_MAX_DATA_LENGTH];
  /** length of the response data */
  uint8_t data_length;
};


/**
 * Describes the Zigbee receive response
 */
struct rflink__zb_rx_response {
  /** 64 bit address (MSB) */
  uint32_t address64_msb;
  /** 64 bit address (LSB) */
  uint32_t address64_lsb;
  /** 16 bit address */
  uint16_t address16;
  /**
   * see XBee.h for possible receive options
   *
   * In addition, ZB_RX_RESPONSE_PAYLOAD_TOO_LARGE denotes data that
   * didn't fit into the frame.
   */
  uint8_t receive_options;
  /** physical copy of the response data */
  uint8_t data[CONFIG_LIBRCLINK_RX_MAX_PAYLOAD];
  /** length of the response data */
  uint8_t data_length;
};


/** Describes the receiver ID packet */
struct rflink__receiver_id {
  uint32_t id;
};


/**
 * Creates a new instace of the rflink
 *
 * @param *xbee_uart - uart used for communicating with the XBee
 * device
 *
 * @return new rflink instance or NULL upon failure
 */
struct rflink* rflink__new(struct uart *xbee_uart);


/**
 * Initializes the transmitter.
 *
 * This method initializes the transmitter and waits for the receiver
 * to connect to the network and identify itself.
 *
 * @param *self - this RC link instance
 *
 * @return RFLINK_SUCCESS if it has been successfully initialized
 */
int rflink__init_transmitter(struct rflink *self);


/**
 * Provides the received signal strength in dBm
 *
 * @param *self - this RC link instance
 * @param *rssi - output parameter - received signal strength value in
 * dBm
 *
 * @return RFLINK__SUCCESS if rssi value has been read successfully
 */
int rflink__get_rssi(struct rflink *self, int *rssi);


/**
 * Provides the current operating channel.
 *
 *
 * @param *self - this RC link instance
 * @param *channel - output parameter - current operating
 * channel. Valid range is 0x0b-0x1a. A value of 0 indicates that the
 * device has not joined any network.
 *
 * @return RFLINK_SUCCESS if the current channel value has been read
 * successfully
 */
int rflink__get_channel(struct rflink *self, int *channel);


/**
 * Handles any incoming frame and submits it for further processing
 *
 * This method is the core of processing of incoming frames. It should
 * be called from a dedicated thread in user code in an infinite loop.
 *
 * @param *self - this RC link instance
 */
void rflink__handle_rx_frame(struct rflink *self);


/**
 * Receive data frame
 *
 * @param *self - this RC link instance
 * @param *buf - buffer of at least MIN_RX_BUF_SIZE for receiving the data
 * @param timeout - time out for the Rx operation
 *
 * @return RFLINK_SUCCESS when data has been successfully sent
 */
int rflink__recv(struct rflink *self, void *buf, portTickType timeout);


/**
 * Sends data to the destination
 *
 * @param *self - this RC link instance
 * @param *buf - buffer to be sent via the link
 * @param len - length of the buffer
 * @param timeout - timeout for the Tx operation
 * @param *delivery_status - optional output parameter - contains
 * delivery status from the link layer
 *
 * @return RFLINK_SUCCESS when data has been successfully sent
 */
int rflink__send(struct rflink *self, const void *buf, size_t len,
		 portTickType timeout, uint8_t *delivery_status);


/**
 * Extracts the data payload from the response
 *
 * @param *buf - pointer to the received frame
 *
 * @return pointer to the data payload
 */
static inline void* rflink__get_response_payload(void *buf)
{
  return (void*)((struct rflink__zb_rx_response*)buf)->data;
}


/**
 * Extracts the data payload length from the response
 *
 * @param *buf - pointer to the received frame
 *
 * @return length of the received data
 */
static inline size_t rflink__get_response_payload_length(void *buf)
{
  return ((struct rflink__zb_rx_response*)buf)->data_length;
}

#endif /* _RFLINK_H_ */
