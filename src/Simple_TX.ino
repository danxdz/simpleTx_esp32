
/*
// Simple Arduino trasmisster
// Arduino Nano
// ELRS 2.4G TX moduel
// Custom PCB from JLCPCB
// const float codeVersion = 0.8; // Software revision
// https://github.com/kkbin505/Arduino-Transmitter-for-ELRS

 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
/* 
 =======================================================================================================
 BUILD OPTIONS (comment out unneeded options)
 =======================================================================================================
 */



//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

#include "config.h"
#include "crsf.c"
#include "led.h"
#include "battery.h"
#include "esp32-hal-uart.h"

static HardwareSerial myser;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t clickCurrentMicros = 0;

    
static uint8_t  currentPower = 0 ;//  "10mW", "25mW", "50mW", "100mW", "250mW"

int Aileron_value = 0;        // values read from the pot 
int Elevator_value = 0; 
int Throttle_value=0;
int Rudder_value = 0; 

int Arm = 0;        // switch values read from the digital pin
int FlightMode = 0; 

float batteryVoltage;


const int DigitalInPinPowerChange = 4;  // 

int powerButtonPressed=0;

bool powerChangeHasRun = false;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

String inputString;
bool stringComplete = false;

void setup() {
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = RC_CHANNEL_MIN;
    }
   //analogReference(EXTERNAL);
   pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
   pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
   pinMode(DIGITAL_PIN_LED, OUTPUT);//LED
   pinMode(DIGITAL_PIN_BUZZER, OUTPUT);//LED
   //digitalWrite(DIGITAL_PIN_BUZZER, LOW);
   batteryVoltage=7.0; 
   
   delay(3000); //Give enough time for uploda firmware
   myser.begin(SERIAL_BAUDRATE,SERIAL_8N1,13, 13,false, 500);
   ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)13, GPIO_MODE_INPUT));
   gpio_matrix_in((gpio_num_t)13, U1RXD_IN_IDX, false);
   gpio_pullup_en((gpio_num_t)13);
   gpio_pulldown_dis((gpio_num_t)13);
   gpio_matrix_out((gpio_num_t)13, U1TXD_OUT_IDX, false, false);


   digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
   

}

    
void loop() {
    uint32_t currentMicros = micros();
    
    batteryVoltage=readVoltage();

    if (batteryVoltage<WARNING_VOLTAGE){
       fastBlinkLED(DIGITAL_PIN_LED);
    }
 // fastBlinkLED(DIGITAL_PIN_LED);
    /*
     * Here you can modify values of rcChannels
     */
    Aileron_value = analogRead(analogInPinAileron); //My gimbal do not center, this function constrain end.
    Elevator_value= analogRead(analogInPinElevator); 
    Throttle_value=analogRead(analogInPinThrottle); 
    Rudder_value = analogRead(analogInPinRudder);  //My gimbal do not center, this function constrain end.
    Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    FlightMode = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    rcChannels[0] = map(Aileron_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX); //reverse
    rcChannels[1] = map(Elevator_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX); 
    rcChannels[2] = map(Throttle_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX);//reverse
    rcChannels[3] = map(Rudder_value ,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
	
	//Aux 1 Arm Channel
    if(Arm==0){
      rcChannels[4] =RC_CHANNEL_MIN;
      //Serial.println("arm");
    }else if(Arm==1){
      //Serial.println("arm-1");

      rcChannels[4] =RC_CHANNEL_MAX;
    }

	//Aux 2 Mode Channel
    if(FlightMode==1){
      rcChannels[5] =RC_CHANNEL_MIN;
    }else if(FlightMode==0){
      rcChannels[5] =RC_CHANNEL_MAX;
    }
    powerButtonPressed = digitalRead(DigitalInPinPowerChange);
    if(powerButtonPressed==0){
      // Setup ELRS Module
      //Serial.println("pwr bt: ");
      if(powerChangeHasRun==false){
        Serial.printf("pwr: %u",currentPower);

        buildElrsPacket(crsfCmdPacket,0,1);
        Serial.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
        delay(4);
        //buildElrsPacket(crsfCmdPacket,17,1);
        //Serial.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
        //delay(4);

        powerChangeHasRun=true;
        clickCurrentMicros = crsfTime + 500000;
      }
    }
	//Additional switch add here.
//  transmit_enable=!digitalRead(transmit_pin);
        
    if (currentMicros > crsfTime) {
        crsfPreparePacket(crsfPacket, rcChannels);
        if (powerChangeHasRun==true && clickCurrentMicros < crsfTime) {
          powerChangeHasRun = false;
          clickCurrentMicros = micros();
        Serial.println("reseT"); 
          
        }
      //For gimal calibation only
	  #ifdef DEBUG
        Serial.printf("curTime: %u - crsfTime: %u - clickTime: %u",currentMicros,crsfTime,clickCurrentMicros); 
        Serial.println("");
      
      /* 
        Serial.print("A_"); 
        Serial.print(Aileron_value); 
        Serial.print("_"); 
        Serial.print(rcChannels[0]); 
        Serial.print(";E_"); 
        Serial.print(Elevator_value);
        Serial.print("_"); 
        Serial.print(rcChannels[1]);  
        Serial.print(";T_"); 
        Serial.print(Throttle_value);
        Serial.print("_"); 
        Serial.print(rcChannels[2]); 
        Serial.print("_R_");  
        Serial.print(Rudder_value);
        Serial.print("_"); 
        Serial.print(rcChannels[3]); 
        Serial.print(";Arm_");  
        Serial.print(Arm);
        Serial.print(";Mode_");  
        Serial.print(FlightMode);
        Serial.print("_BatteryVoltage:");  
        Serial.print(batteryVoltage);
        Serial.println();*/  
        delay(1000);
       #else
         myser.write(crsfPacket, CRSF_PACKET_SIZE);

	   #endif
        if (stringComplete) {
    Serial1.println("******************************************************");
    //Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
    serialEvent();
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
 Serial1.println("**--------------------------***");
 
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    //Serial.println("**--------------------------***");

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// ESP32 Team900
        // buildElrsPacket(crsfCmdPacket,X,3);
        // 0 : ELRS status request => ??
        // 1 : Set Lua [Packet Rate]= 0 - 25Hz / 1 - 50Hz / 2 - 100Hz / 3 - 200Hz
        // 2 : Set Lua [Telem Ratio]= 0 - off / 1 - 1:128 / 2 - 1:64 / 3 - 1:32 / 4 - 1:16 / 5 - 1:8 / 6 - 1:4 / 7 - 1:2
        // 3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide 
        // 4 : Set Lua [Model Match]=0 -> Off;On
        // 5 : Set Lua [TX Power]=0 - 10mW / 1 - 25mW
        // 6 : Set Lua [Max Power]=0 - 10mW / 1 - 25mW *dont force to change, but change after reboot if last power was greater
        // 7 : Set Lua [Dynamic]=0 -> Off;On;AUX9;AUX10;AUX11;AUX12 -> * @ ttgo screen
        // 8 : Set Lua [VTX Administrator]=0
        // 9 : Set Lua [Band]=0 -> Off;A;B;E;F;R;L
        // 10: Set Lua [Channel]=0 -> 1;2;3;4;5;6;7;8
        // 11: Set Lua [Pwr Lvl]=0 -> -;1;2;3;4;5;6;7;8
        // 12: Set Lua [Pitmode]=0 -> Off;On
        // 13: Set Lua [Send VTx]=0 sending response for [Send VTx] chunk=0 step=2
        // 14: Set Lua [WiFi Connectivity]=0
        // 15: Set Lua [Enable WiFi]=0 sending response for [Enable WiFi] chunk=0 step=0
        // 16: Set Lua [Enable Rx WiFi]=0 sending response for [Enable Rx WiFi] chunk=0 step=2
        // 17: Set Lua [BLE Joystick]=0 sending response for [BLE Joystick] chunk=0 step=0
        //     Set Lua [BLE Joystick]=1 sending response for [BLE Joystick] chunk=0 step=3
        //     Set Lua [BLE Joystick]=2 sending response for [BLE Joystick] chunk=0 step=3
        // 19: Set Lua [Bad/Good]=0
        // 20: Set Lua [2.1.0 EU868]=0 =1 ?? get 
