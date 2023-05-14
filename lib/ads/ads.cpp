#include "Arduino.h"
#include "ads.h"
#include <Adafruit_ADS1X15.h>
#include "gpio.h"
#include "ui_buttons.h"
#include "config.h"
#include "uart.h"

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

void ADSreader::init()
{
#if !defined(DEBUG)
    dbout.printf("starting ads...\n");
#endif

    u_int32_t i2c_addr = 0x48;

    if (!ads.begin(i2c_addr))
    {
        dbout.println("Failed to initialize ADS.");
        while (1);
    }
    else
    {
         // Configura o ganho para 2/3
        ads.setGain(GAIN_TWOTHIRDS);

        dbout.println("ADS initialized.");
    }

    delay(500);
}

void ADSreader::readInputs(rc_input_t *rc_input)
{
    // read the analog in value:
    // converte float to int
    int16_t a0 = (ads.readADC_SingleEnded(0));
    int16_t a1 = (int16_t)(ads.readADC_SingleEnded(1));
    int16_t a2 = (int16_t)(ads.readADC_SingleEnded(2));
    float a3 = (ads.readADC_SingleEnded(3));
    // map it to the range of the analog out:
    int16_t a0Scaled = a0 >> 4;
    int16_t a1Scaled = a1 >> 4;
    int16_t a2Scaled = a2 >> 4;

    rc_input->aileron = a0;
    rc_input->elevator = a1;
    rc_input->throttle = a2;
    rc_input->rudder = a3;

    if (!debugEnabled)
    {   
        float Voltage0 = (a0 * 0.1875) / 1000;
        float Voltage1 = (a1 * 0.1875) / 1000;
        float Voltage2 = (a2 * 0.1875) / 1000;
        float Voltage3 = (a3 * 0.1875) / 1000;
        dbout.printf("%f :: %f :: %f :: %f\n", Voltage0, Voltage1, Voltage2, Voltage3); 
    }
    

#if defined(debug)
    dbout.printf("aileron: %d\n", rc_input->aileron);
    dbout.printf("elevator: %d\n", rc_input->elevator);
    dbout.printf("throttle: %d\n", rc_input->throttle);
    dbout.printf("rudder: %d\n", rc_input->rudder);
#endif

    rc_input->aux1 = analogRead(ANALOG_IN_PIN_AUX1);
    rc_input->aux2 = analogRead(ANALOG_IN_PIN_AUX2);
    // rc_input->aux3 = analogRead(ANALOG_IN_PIN_AUX3);
    // rc_input->aux4 = analogRead(ANALOG_IN_PIN_AUX4);
}