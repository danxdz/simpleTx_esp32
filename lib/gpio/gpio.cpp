#include "Arduino.h"
#include "config.h"
#include "gpio.h"
#include "crsf.h"

// IO setup

int upBt = 12;
int downBt = 4;
int enterBt = 14;
int backBt = 5;

void initGpio()
{

  // analogReference(EXTERNAL);
  pinMode(ANALOG_IN_PIN_AUX1, INPUT_PULLDOWN);
  pinMode(ANALOG_IN_PIN_AUX2, INPUT_PULLUP);
  // pinMode(DIGITAL_PIN_LED, OUTPUT);//LED
  // pinMode(DIGITAL_PIN_BUZZER, OUTPUT);//
  // digitalWrite(DIGITAL_PIN_BUZZER, LOW);
  // batteryVoltage=7.0;

  pinMode(upBt, INPUT_PULLUP);
  pinMode(downBt, INPUT_PULLUP);
  pinMode(enterBt, INPUT_PULLUP);
  pinMode(backBt, INPUT_PULLUP);
  pinMode(DigitalInPinPowerChange, INPUT_PULLUP);

  #if defined(TARGET_ESP32_S)
    analogReadResolution(12); // 4096
  #endif

}

void gpioReadInputs(rc_input_t* rc_input)
{
    rc_input->aileron  = analogRead(ANALOG_IN_PIN_AILERON);
    rc_input->elevator = analogRead(ANALOG_IN_PIN_ELEVATOR);
    rc_input->throttle = analogRead(ANALOG_IN_PIN_THROTTLE);
    rc_input->rudder   = analogRead(ANALOG_IN_PIN_RUDDER);
    rc_input->aux1     = analogRead(ANALOG_IN_PIN_AUX1);
    rc_input->aux2     = analogRead(ANALOG_IN_PIN_AUX2);
    rc_input->aux3     = analogRead(ANALOG_IN_PIN_AUX3);
    rc_input->aux4     = analogRead(ANALOG_IN_PIN_AUX4);
}

void gpioMixer(rc_input_t* rc_input)
{
    rc_input->aux1 = (rc_input->aux1 >= 2047) ? 4095 : 0;
}
