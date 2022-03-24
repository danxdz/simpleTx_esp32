/*
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
 Simple TX CONFIG OPTIONS (comment out unneeded options)
 =======================================================================================================
 */
int Aileron_value = 0;        // values read from the pot 
int Elevator_value = 0; 
int Throttle_value=0;
int Rudder_value = 0; 

int Arm = 0;        // switch values read from the digital pin
int FlightMode = 0; 

 // Define RC input Offset
int Aileron_OFFSET = 0;        // values read from the pot 
int Elevator_OFFSET  = 0; 
int Throttle_OFFSET =0;
int Rudder_OFFSET  = 0; 

//IO setup
//pins that used for the Joystick
const int analogInPinAileron = 32;
const int analogInPinElevator = 33; 
const int analogInPinThrottle = 34;
const int analogInPinRudder = 35; 
const int VOLTAGE_READ_PIN = 36; 


//pins that used for the switch
const int DIGITAL_PIN_SWITCH_ARM = 0;  // Arm switch
const int DIGITAL_PIN_SWITCH_AUX2 = 2;  // 

//pins that used for output
const int DIGITAL_PIN_LED = 16;  // 
const int DIGITAL_PIN_BUZZER = 12;  // 

//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 //get average of VOLTAGE_READS readings

 // Define battery warning voltage
 const float WARNING_VOLTAGE=7.2; //2S Lipo

static HardwareSerial elrs(1);
static HardwareSerial db_out(0);
