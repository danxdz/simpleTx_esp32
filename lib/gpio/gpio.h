#ifndef GPIO_H /* include guards */
#define GPIO_H

//IO setup
//pins that used for the Joystick
const int analogInPinAileron = 32;
const int analogInPinElevator = 33; 
const int analogInPinThrottle = 34;
const int analogInPinRudder = 35; 
const int VOLTAGE_READ_PIN = 36; 


//pins that used for the switch
const int DIGITAL_PIN_SWITCH_ARM = 16;  // Arm switch
const int DIGITAL_PIN_SWITCH_AUX2 = 2;  // 

//pin for button : testing bt
const int DigitalInPinPowerChange = 15;  // 
//button bouncer
static int testButtonPressed;

///const int upBt = 12;
//const int downBt = 4;
const int enterBt = 0;
const int backBt = 5;


//pins that used for output
const int DIGITAL_PIN_LED = 16;  // 
const int DIGITAL_PIN_BUZZER = 12;  // 

//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 //get average of VOLTAGE_READS readings

 // Define battery warning voltage
 const float WARNING_VOLTAGE=7.2; //2S Lipo


void initGpio ();

#endif