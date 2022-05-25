#ifndef GPIO_H /* include guards */
#define GPIO_H

//IO setup



//pin for button : testing bt
const int DigitalInPinPowerChange = 15;  // 
//button bouncer
static int testButtonPressed;

extern int upBt;
extern int downBt;
extern int enterBt;
extern int backBt;


//pins that used for output
const int DIGITAL_PIN_LED = 16;  // 
const int DIGITAL_PIN_BUZZER = 12;  // 

//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 //get average of VOLTAGE_READS readings

 // Define battery warning voltage
 const float WARNING_VOLTAGE=7.2; //2S Lipo


void initGpio ();

int *get_rc_channels();

#endif