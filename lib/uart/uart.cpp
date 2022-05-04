#include <Arduino.h>
#include <HardwareSerial.h>

#include "uart.h"
#include "crsf_protocol.h"

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

crsfPayloadLinkstatistics_s LinkStatistics; 

volatile crsf_sensor_battery_s batteryVoltage;

HardwareSerial dbout(0);
HardwareSerial elrs(1);
