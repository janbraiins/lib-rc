#ifndef _RCLINK_XBEE_WRAPPER_H_
#define _RCLINK_XBEE_WRAPPER_H_

/* Arduino ZigBee stack */
#include <XBee.h>


/**
 * This class serves as an internal wrapper around the xbee instance
 * and responses/requests used for fast processing of incoming ZigBee
 * frames. It is intended for exclusive use by the rclink module.
 */
class RCLinkXBeeWrapper {
private:
  /** XBee instance associated with this link */
  XBee xbee;

  /* The following items are provided to rx/tx handlers so that the
   * instances of each type won't get created every time the handler
   * method is called. This eliminates calling constructors for every
   * received frame and results in better performance. The data from
   * the responses is extracted into more suitable form so that it can
   * be submitted for further processing via queues */

  /** the most recent modem status response */
  ModemStatusResponse modem_status_response;
  /** the most recent AT command response */
  AtCommandResponse at_command_response;
  /** the most recent ZigBee Rx response */
  ZBRxResponse zb_rx_response;
  /** the most recent ZigBee Tx request  */
  ZBTxRequest zb_tx_request;
  /** the most recent ZigBee Tx Status response */
  ZBTxStatusResponse zb_tx_status_response;
  /** AT command request */
  AtCommandRequest at_command_request;

public:
  /** 
   * Fetches the current status response from the XBee stack and
   * provides the cached instance to the user
   *
   * @return modem status instance reference
   */
  inline ModemStatusResponse& getModemStatusResponse()
  {
    xbee.getResponse().getModemStatusResponse(modem_status_response);
    return modem_status_response;
  }

  /** 
   * Fetches the current AT command response from the XBee stack and
   * provides the cached instance to the user
   *
   * @return AT command response
   */
  inline AtCommandResponse& getAtCommandResponse()
  {
    xbee.getResponse().getAtCommandResponse(at_command_response);
    return at_command_response;
  }


  /** 
   * Fetches the current ZigBee Tx status response from the XBee stack and
   * provides the cached instance to the user
   *
   * @return ZigBee Tx status response
   */
  inline ZBTxStatusResponse& getZBTxStatusResponse()
  {
    xbee.getResponse().getZBTxStatusResponse(zb_tx_status_response);
    return zb_tx_status_response;
  }

  /** 
   * Fetches the current ZigBee Rx response from the XBee stack and
   * provides the cached instance to the user
   *
   * @return ZigBee Rx response
   */
  inline ZBRxResponse& getZBRxResponse()
  {
    xbee.getResponse().getZBRxResponse(zb_rx_response);
    return zb_rx_response;
  }

  /**
   *
   * @return XBee stack instance
   */
  inline XBee& getXBee()
  {
    return xbee;
  }

  inline void send(XBeeRequest &request)
  {
    xbee.send(request);
  }

  /**
   * @param at_command[] - 2 byte sequence containing the AT command
   * @param at_command_data[] - optional data payload
   * @param at_command_data_length - length of the data payload
   */
  inline void submitAtCommandRequest(const uint8_t at_command[],
				     const uint8_t at_command_data[],
				     uint8_t at_command_data_length)
  {
    at_command_request.setCommand((uint8_t*)at_command);

    if (at_command_data_length != 0) {
      at_command_request.setCommandValue((uint8_t*)at_command_data);
      at_command_request.setCommandValueLength(at_command_data_length);
    }

    send(at_command_request);
  }

  /**
   * Sends a ZigBee Tx request to the modem
   *
   * @param *buf - payload being submitted
   * @param len - length of the payload in bytes
   */
  inline void submitZBTxRequest(const void *buf, size_t len)
  {
    zb_tx_request.setPayload((uint8_t*)buf);
    zb_tx_request.setPayloadLength((uint8_t)len);

    send(zb_tx_request);
  }


  /**
   * Sets the destination address in Tx Request
   *
   * @param addr64_msb - msb part of the address
   * @param addr64_lsb - lsb part of the address
   */
  inline void setZBTxDestAddress64(uint32_t addr64_msb, uint32_t addr64_lsb)
  {
    XBeeAddress64 addr64(addr64_msb, addr64_lsb);
    zb_tx_request.setAddress64(addr64);
    zb_tx_request.setAddress16(ZB_BROADCAST_ADDRESS);
  }
};

#endif /* _RCLINK_XBEE_WRAPPER_H_ */
