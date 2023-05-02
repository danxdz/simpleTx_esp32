#include "Arduino.h"
#include "config.h"
#include "gpio.h"
#include "ui_buttons.h"
#include "crsf.h"
#include "uart.h"


void initGpio()
{

  // analogReference(EXTERNAL);
  pinMode(ANALOG_IN_PIN_AUX1, INPUT_PULLDOWN);
  pinMode(ANALOG_IN_PIN_AUX2, INPUT_PULLUP);
  // pinMode(DIGITAL_PIN_LED, OUTPUT);//LED
  // pinMode(DIGITAL_PIN_BUZZER, OUTPUT);//
  // digitalWrite(DIGITAL_PIN_BUZZER, LOW);
  // batteryVoltage=7.0;

  pinMode(UI_BTN_UP, INPUT_PULLUP);
  pinMode(UI_BTN_DOWN, INPUT_PULLUP);
  pinMode(UI_BTN_ENTER, INPUT_PULLUP);
  pinMode(UI_BTN_BACK, INPUT_PULLUP);
  

  #if defined(TARGET_ESP32_S)
    analogReadResolution(12); // 4096
  #endif

}



void readUIbuttons (UI_input_t* UI_input) {

  UI_input->up = digitalRead(UI_BTN_UP);
  UI_input->down = digitalRead(UI_BTN_DOWN);
  UI_input->enter = digitalRead(UI_BTN_ENTER);
  UI_input->back = digitalRead(UI_BTN_BACK);

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
