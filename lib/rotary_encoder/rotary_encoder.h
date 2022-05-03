#ifndef ROTARY_ENCODER_H /* include guards */
#define ROTARY_ENCODER_H

#include <ESP32Encoder.h>


static ESP32Encoder encoder;
static int rotary_encoder_last_pos;

uint8_t encoderInit();
uint8_t getRE_POS();

#endif