#include <Arduino.h>
#include <HardwareSerial.h>

#include "uart.h"
#include "crsf_protocol.h"


bool debugEnabled = true; // Defina como 'true' para ativar a depuração

// portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial dbout(0);
HardwareSerial elrs(1);
