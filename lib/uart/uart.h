#ifndef UART_H /* include guards */
#define UART_H

#include "Arduino.h"
#include "crsf_protocol.h"


//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
extern HardwareSerial elrs;
extern HardwareSerial dbout;

extern volatile crsf_sensor_battery_s batteryVoltage;
extern crsfPayloadLinkstatistics_s LinkStatistics; // Link Statisitics Stored as Struct

#endif