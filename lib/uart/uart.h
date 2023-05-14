#ifndef UART_H /* include guards */
#define UART_H

#include "Arduino.h"
#include "crsf_protocol.h"

extern HardwareSerial elrs;
extern HardwareSerial dbout;

extern bool debugEnabled; // Variável de controle da depuração

#endif
