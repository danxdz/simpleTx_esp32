#pragma once

#include <Arduino.h>



// IO setup

// pin for button : testing bt
//const int DigitalInPinPowerChange = 15; //
// button bouncer
static int testButtonPressed;

// pins that used for output
//const int DIGITAL_PIN_LED = 16;    //
//const int DIGITAL_PIN_BUZZER = 12; //


//pins that used for the Joystick
#if defined(TARGET_ESP32)

  #define ANALOG_IN_PIN_ELEVATOR 32
  #define ANALOG_IN_PIN_AILERON 33
  #define ANALOG_IN_PIN_THROTTLE 34
  #define ANALOG_IN_PIN_RUDDER 35
  #define ANALOG_IN_PIN_AUX1 25  // Arm switch
  #define ANALOG_IN_PIN_AUX2 26  // Mode switch
  #define ANALOG_IN_PIN_AUX3 27  // gpio27
  #define ANALOG_IN_PIN_AUX4 12  // gpio12
  #define VOLTAGE_READ_PIN 36

  // UI buttons
  #define UI_BTN_UP 15
  #define UI_BTN_DOWN 4
  #define UI_BTN_ENTER 14
  #define UI_BTN_BACK 5
  #define UI_KEY_PAD 39


#elif defined(TARGET_ESP32_S)

  #define ANALOG_IN_PIN_ELEVATOR A2 // gpio3
  #define ANALOG_IN_PIN_AILERON A3 // gpio4
  #define ANALOG_IN_PIN_THROTTLE A4 // gpio5
  #define ANALOG_IN_PIN_RUDDER A5 // gpio6
  #define ANALOG_IN_PIN_AUX1 A10  // gpio11 - Arm switch
  #define ANALOG_IN_PIN_AUX2 A11 // gpio12 - Mode switch
  #define ANALOG_IN_PIN_AUX3 A12  // gpio13
  #define ANALOG_IN_PIN_AUX4 A13  // gpio14
  #define VOLTAGE_READ_PIN A0 // gpio0

#endif

extern bool powerChangeHasRun;

extern uint32_t tickTime;
extern uint16_t rates[];
// click deboucer
static uint32_t clickCurrentMicros = 0;

#define TEMPSTRINGLENGTH 400 // This is the max dialog size (80 characters * 5 lines)
                             // We could reduce this to ~240 on the 128x64 screens
                             // But only after all sprintf are replaced with snprintf
                             // Maybe move this to target_defs.h
extern char tempstring[TEMPSTRINGLENGTH];

void check_link_state(uint32_t currentMicros);
const char *hdr_str_cb(const void *data);
void crsfdevice_init();
void bt_handle(uint8_t value);