#include <rclink.h>

/**
 * maps Tx deliver status codes to string
 */
struct status_map tx_delivery_status_map[] = {
  {SUCCESS, "success"},
  {CCA_FAILURE}, "CCA failure"},
  {INVALID_DESTINATION_ENDPOINT_SUCCESS, "Invalid destination endpoint"},
  {NETWORK_ACK_FAILURE, "Network ACK failure"},
  {NOT_JOINED_TO_NETWORK, "Not joined to network"},
  {SELF_ADDRESSED, "Self-addressed"},
  {ADDRESS_NOT_FOUND, "Address not found"},
  {ROUTE_NOT_FOUND, "route not found"},
  {PAYLOAD_TOO_LARGE, "Payload too large"},
  {0, NULL},
};


/**
 * maps modem status codes to string
 */
struct status_map modem_status_map[] = {
  {HARDWARE_RESET, "Hardware reset"},
  {WATCHDOG_TIMER_RESET, "Watchdog timer reset"},
  {ASSOCIATED, "Associated"},
  {DISASSOCIATED, "Disassociated"},
  {SYNCHRONIZATION_LOST, "Synchronization lost"},
  {COORDINATOR_REALIGNMENT, "Coordinator realignment"},
  {COORDINATOR_STARTED, "Coordinator started"},
  {0, NULL},
};
