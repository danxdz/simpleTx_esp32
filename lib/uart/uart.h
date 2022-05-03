#ifndef UART_H /* include guards */
#define UART_H

#include "Arduino.h"
#include "crsf_protocol.h"

static crsfPayloadLinkstatistics_s LinkStatistics; // Link Statisitics Stored as Struct
static volatile crsf_sensor_battery_s batteryVoltage;

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static HardwareSerial db_out(0);

#endif