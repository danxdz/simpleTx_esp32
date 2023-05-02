#ifndef GPIO_H /* include guards */
#define GPIO_H
#include "crsf.h"





//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 // get average of VOLTAGE_READS readings

// Define battery warning voltage
const float WARNING_VOLTAGE = 7.2; // 2S Lipo

void initGpio();
void gpioReadInputs(rc_input_t* rc_input);
void gpioMixer(rc_input_t* rc_input);

#endif