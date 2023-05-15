#include <Arduino.h>
#include <HardwareSerial.h>

#include "uart.h"
#include "crsf_protocol.h"

bool DEBUG_PACKETS = true;
// #define DEBUG_TLM
// #define DEBUG_CH
// #define DEBUG_SYNC
// #define DEBUG_HALF_DUPLEX
// #define DEBUG_CRSF_FRAMETYPE_RADIO_ID

bool debugEnabled = true; // Set 'true' to actvate debug output on UART0 / dbout

// portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial dbout(0);
HardwareSerial elrs(1);
