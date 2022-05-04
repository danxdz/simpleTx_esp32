#include "Arduino.h"
#include "gpio.h"

int upBt = 12;
int downBt = 4;
int enterBt = 0;
int backBt = 5;

void initGpio () {



//analogReference(EXTERNAL);
  pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLDOWN);
  pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
  //pinMode(DIGITAL_PIN_LED, OUTPUT);//LED
  //pinMode(DIGITAL_PIN_BUZZER, OUTPUT);//
  //digitalWrite(DIGITAL_PIN_BUZZER, LOW);
  //batteryVoltage=7.0; 

  pinMode(upBt, INPUT_PULLUP);
  pinMode(downBt, INPUT_PULLUP);
  pinMode(enterBt, INPUT_PULLUP);
  pinMode(backBt, INPUT_PULLUP);

}

