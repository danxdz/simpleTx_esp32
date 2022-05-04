#ifndef ROTARY_ENCODER_H /* include guards */
#define ROTARY_ENCODER_H

#include <ESP32Encoder.h>


static ESP32Encoder encoder;
extern int rotary_encoder_last_pos;

uint8_t encoderInit();
uint8_t get_encoder_pos();

#endif