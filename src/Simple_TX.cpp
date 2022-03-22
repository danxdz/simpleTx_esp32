
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

//#define DEBUG_PACKETS// if not commented out, Serial.print() is active! For debugging only!!
//#define DEBUG_TLM // if not commented out, Serial.print() is active! For debugging only!!
//#define DEBUG_CH // if not commented out, Serial.print() is active! For debugging only!!
//#define DEBUG_SYNC // if not commented out, Serial.print() is active! For debugging only!!
//#define DEBUG_HALF_DUPLEX // if not commented out, Serial.print() is active! For debugging only!!


#include <Arduino.h>
#include <HardwareSerial.h>

#include "config.h"
#include "crsf.c"
#include "led.h"
#include "battery.h"
#include "tlm.h"
#include "CrsfProtocol/crsf_protocol.h"

#include "oled.h"
#include "halfduplex.h"


static HardwareSerial elrs(1);
static HardwareSerial db_out(0);

TaskHandle_t Task1;
TaskHandle_t Task2;

/// UART Handling ///
uint32_t GoodPktsCount = 0;
uint32_t BadPktsCount = 0;
uint32_t UARTwdtLastChecked;
/// UART Handling ///
volatile uint8_t SerialInPacketLen = 0; // length of the CRSF packet as measured
volatile uint8_t SerialInPacketPtr = 0; // index where we are reading/writing
volatile bool CRSFframeActive = false; //since we get a copy of the serial data use this flag to know when to ignore it

static volatile crsfPayloadLinkstatistics_s LinkStatistics; // Link Statisitics Stored as Struct
static volatile crsf_sensor_battery_s batteryVoltage; // Link Statisitics Stored as Struct

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t currentMicros = 0;
uint32_t clickCurrentMicros = 0;//click deboucer
uint32_t displayCurrentMicros = 0;//display deboucer
    

//float batteryVoltage;

const int DigitalInPinPowerChange = 4;  // 

int powerButtonPressed=0;
bool powerChangeHasRun = false;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];

uint32_t crsfTime = 0;
uint32_t lastCrsfTime = 0;

static uint8_t SerialInBuffer[CRSF_MAX_PACKET_LEN];
//static u8 telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];

static uint32_t updateInterval = CRSF_TIME_BETWEEN_FRAMES_US;
static int32_t correction;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void CRSF_serial_rcv(uint8_t *buffer, uint8_t num_bytes) {
    db_out.printf("%u",buffer[0]);
    db_out.println("");
    switch (buffer[0]) {
    case CRSF_FRAMETYPE_DEVICE_INFO:
        db_out.printf("1: %u ; %u",buffer[0],num_bytes);
        db_out.println("");
        break;

    case CRSF_FRAMETYPE_ELRS_STATUS:
        //parse_elrs_info(buffer);
        db_out.printf("2: %u ; %u",buffer[0],num_bytes);
        db_out.println("");
    
        break;

    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        db_out.printf("3: %u ; %u",buffer[0],num_bytes);
        db_out.println("");
    
        //read_timeout = 0;
        //add_param(buffer, num_bytes);
        break;

    default:
        break;
    } 
}


static uint32_t get_update_interval() {
    if (correction == 0) return updateInterval;

    uint32_t update = updateInterval + correction;
    update = constrain(update, CRSF_FRAME_PERIOD_MIN, CRSF_FRAME_PERIOD_MAX);
    correction -= update - updateInterval;
    return update;
}

static uint8_t getCrossfireTelemetryValue(uint8_t index, int32_t *value, uint8_t len) {
  uint8_t result = 0;
  uint8_t *byte = &SerialInBuffer[index];
  *value = (*byte & 0x80) ? -1 : 0;
  for (int i=0; i < len; i++) {
    *value <<= 8;
    if (*byte != 0xff) result = 1;
    *value += *byte++;
  }
  return result;
}
void serialEvent() {
  
  while (elrs.available()) {
    if (CRSFframeActive == false)
    {
      unsigned char const inChar = elrs.read();
      // stage 1 wait for sync byte //
      //db_out.write(inChar);
      if (inChar == CRSF_ADDRESS_RADIO_TRANSMITTER)
      {
        // we got sync, reset write pointer
        SerialInPacketPtr = 0;
        SerialInPacketLen = 0;
        CRSFframeActive = true;
        SerialInBuffer[SerialInPacketPtr] = inChar;
        SerialInPacketPtr++;
      }
    } else // frame is active so we do the processing
    {
      // first if things have gone wrong //
      if (SerialInPacketPtr > CRSF_MAX_PACKET_LEN - 1)
      {
        // we reached the maximum allowable packet length, so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        SerialInPacketLen = 0;
        CRSFframeActive = false;
        db_out.println("bad packet len");
        return;
      }
      // special case where we save the expected pkt len to buffer //
      if (SerialInPacketPtr == 1)
      {
        unsigned char const inChar = elrs.read();

        if (inChar <= CRSF_MAX_PACKET_LEN)
        {
            SerialInPacketLen = inChar;
            SerialInBuffer[SerialInPacketPtr] = inChar;
            SerialInPacketPtr++;
        }
        else
        {
            SerialInPacketPtr = 0;
            SerialInPacketLen = 0;
            CRSFframeActive = false;
            return;
        }
      }

      int toRead = (SerialInPacketLen + 2) - SerialInPacketPtr;
      int count = elrs.read(&SerialInBuffer[SerialInPacketPtr], toRead);
      SerialInPacketPtr += count;
      //////db_out.println(count);

      if (SerialInPacketPtr >= (SerialInPacketLen + 2)) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
      {
        char CalculatedCRC = crsf_crc8(SerialInBuffer + 2, SerialInPacketPtr - 3);
        if (CalculatedCRC == SerialInBuffer[SerialInPacketPtr-1])
        {
          GoodPktsCount++;

            int32_t value;
            uint8_t id = SerialInBuffer[2];

            CRSF_serial_rcv(SerialInBuffer+2, SerialInBuffer[1]-1);


            if (id == CRSF_FRAMETYPE_BATTERY_SENSOR) {
              //db_out.print("battery");
              if (getCrossfireTelemetryValue(3, &value, 2)) {
                batteryVoltage.voltage = value;
              }
            }
            if (id == CRSF_FRAMETYPE_RADIO_ID) {
              //db_out.print("radio id");

              if (SerialInBuffer[3] == CRSF_ADDRESS_RADIO_TRANSMITTER//0xEA - radio address
                && SerialInBuffer[5] == CRSF_FRAMETYPE_OPENTX_SYNC//0x10 - timing correction frame
              ) { 
                if (getCrossfireTelemetryValue(6, (int32_t *)&updateInterval,4) &&
                    getCrossfireTelemetryValue(10,(int32_t *)&correction, 4)) {
                    // values are in 10th of micro-seconds
                    updateInterval /= 10;
                    correction /= 10;
                    if (correction >= 0)
                      correction %= updateInterval;
                    else
                      correction = -((-correction) % updateInterval);
                }
              }
            }

            if (id == CRSF_FRAMETYPE_LINK_STATISTICS) {
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_RX_RSSI1, &value, 1)) { 
                LinkStatistics.uplink_RSSI_1 = value;
              }
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_RX_RSSI2, &value, 1)) { 
                LinkStatistics.uplink_RSSI_2 = value;
              } 
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_RX_QUALITY, &value, 1)) { 
                LinkStatistics.uplink_Link_quality = value;
              } 
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_RF_MODE, &value, 1)) { 
                LinkStatistics.rf_Mode = value;
              } 
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_TX_POWER, &value, 1)) { 
                static const int32_t power_values[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };
                //if ((int8_t)value >= (sizeof power_values / sizeof (int32_t)))
                //  continue;
                value = power_values[value];
                LinkStatistics.uplink_TX_Power = value;
              }
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_TX_RSSI, &value, 1)) { 
                LinkStatistics.downlink_RSSI = value;
              }
              if (getCrossfireTelemetryValue(2+TELEM_CRSF_TX_QUALITY, &value, 1)) { 
                LinkStatistics.downlink_Link_quality = value;
              }
            }
          
          #ifdef DEBUG_TLM
  
          /*   const uint8_t temp = inBuffer.asRCPacket_t.header.frame_size;
          db_out.write(temp);
          const uint8_t packetType = inBuffer.asRCPacket_t.header.type;
          db_out.write(packetType);
          db_out.printf("%u " ,SerialInBuffer[0]);
          db_out.printf("%u " ,SerialInBuffer[1]);
          db_out.printf("%u " ,SerialInBuffer[2]);
          db_out.printf("%u " ,SerialInBuffer[3]);
          db_out.printf("%u " ,SerialInBuffer[4]);
          db_out.printf("%u " ,SerialInBuffer[5]);
          db_out.printf("%u " ,SerialInBuffer[6]);
          db_out.printf("%u " ,SerialInBuffer[7]);
          db_out.printf("%u " ,SerialInBuffer[8]);
          db_out.printf("%u " ,SerialInBuffer[9]);
          db_out.printf("%u " ,SerialInBuffer[10]);
          db_out.printf("%u " ,SerialInBuffer[11]);
          db_out.printf("%u " ,SerialInBuffer[12]);
          db_out.printf("%u " ,SerialInBuffer[13]);
          db_out.printf("%u " ,SerialInBuffer[14]);
          db_out.printf("%u " ,SerialInBuffer[15]);
          db_out.printf("%u " ,SerialInBuffer[16]);
          db_out.println(""); */
        }
          #endif
          #ifdef DEBUG_PACKETS
          //output packets to serial for debug
            for (int i=0;i<=15;i++) {
              db_out.write(SerialInBuffer[i]); 
            }
          #endif
    
        
        } else {
          db_out.write("UART CRC failure");
          // cleanup input buffer
          elrs.flush();
          BadPktsCount++;
        }
        CRSFframeActive = false;
        SerialInPacketPtr = 0;
        SerialInPacketLen = 0;
      }
    }  
  }
}

void OutputTask( void * pvParameters ){
  
  //uart debug
  db_out.begin(115200);
  delay(3000); 

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.display();

  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
  delay(500);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.printf("ELRS");
  display.setTextSize(2);             
  display.println("");
  display.printf("simpleTX");
  display.display();
  delay(2000);

  startDisplay();



  for(;;){

    delay(1000);
      updateDisplay(
        LinkStatistics.downlink_RSSI,
        LinkStatistics.downlink_Link_quality,
        LinkStatistics.rf_Mode,
        LinkStatistics.uplink_TX_Power,
        LinkStatistics.uplink_RSSI_1,
        LinkStatistics.uplink_RSSI_2,
        LinkStatistics.uplink_Link_quality,
        batteryVoltage.voltage,
        200);//get_update_interval());
  } 
}


//Task2 - ELRS task - main loop
void ElrsTask( void * pvParameters ){
  //setup channels
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
    rcChannels[i] = RC_CHANNEL_MIN;
  }
 
  elrs.begin(SERIAL_BAUDRATE,SERIAL_8N1,13, 13,false, 500);
  db_out.write("starting");
  db_out.println("");
  duplex_set_TX();  

  //digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON

  //TODO
  // change to get last rate
  int packetRateSelected = 0;
  for(;;){

  currentMicros = micros();

  // Here you can modify values of rcChannels
  Aileron_value = analogRead(analogInPinAileron); 
  Elevator_value= analogRead(analogInPinElevator); 
  Throttle_value= analogRead(analogInPinThrottle); 
  Rudder_value = analogRead(analogInPinRudder);
  Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
  FlightMode = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
  rcChannels[0] = map(Aileron_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
  rcChannels[1] = map(Elevator_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX); 
  rcChannels[2] = map(Throttle_value,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
  rcChannels[3] = map(Rudder_value ,0,4096,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
	
	//Aux 1 Arm Channel
  if(Arm==1){
    rcChannels[4] = RC_CHANNEL_MIN;
    //Serial.println("arm-0");
  } else if (Arm==0) {
    rcChannels[4] = RC_CHANNEL_MAX;
    //Serial.println("arm-1");
    }
	//Aux 2 Mode Channel
  if(FlightMode==1){
    rcChannels[5] =RC_CHANNEL_MIN;
  } else if(FlightMode==0){
    rcChannels[5] =RC_CHANNEL_MAX;
  }
  
  powerButtonPressed = digitalRead(DigitalInPinPowerChange);
  if(powerButtonPressed==0){
    // Setup ELRS Module
    //Serial.println("pwr bt: ");
    if(powerChangeHasRun==false) {
      //db_out.printf("pwr: %u",currentPower);
      //db_out.println("click");
      if(packetRateSelected>=6) {
        packetRateSelected = 0;
      } else {
        packetRateSelected++;
      }

      buildElrsPacket(crsfCmdPacket,1,packetRateSelected);
      duplex_set_TX();
      elrs.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
      elrs.flush();
      delay(4);
      
      //uncomment to send 2e command
      /* buildElrsPacket(crsfCmdPacket,1,packetRateSelected);
      duplex_set_TX();
      elrs.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
      elrs.flush();
      delay(4);
       */
      
      duplex_set_RX();
      powerChangeHasRun=true;
      clickCurrentMicros = crsfTime + (2*1000000);//2sec
      
      }
    }
	//Additional switch add here.
  
  //  transmit_enable=!digitalRead(transmit_pin); ???
        
  if (currentMicros >= crsfTime) {
  
    crsfPreparePacket(crsfPacket, rcChannels);
    if (powerChangeHasRun==true && clickCurrentMicros < crsfTime) 
    {
      powerChangeHasRun = false;
      clickCurrentMicros = micros();
      //db_out.println("reset"); 
    }
	  #ifdef DEBUG_CH
      //For gimal calibation only
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
      Serial.println();
      delay(1000);
    #else

      //send crsf packet
      duplex_set_TX();  
      elrs.write(crsfPacket, CRSF_PACKET_SIZE);
      elrs.flush();
    
      //start receiving
      duplex_set_RX();
      serialEvent();
      
      crsfTime = currentMicros;//set current micros
      int32_t offset = (crsfTime-lastCrsfTime);//-updated_interval;//get dif between pckt send
      uint32_t updated_interval = get_update_interval();
      //debug timing
      #ifdef DEBUG_SYNC
      if (updated_interval > updateInterval) {
        db_out.printf("%u ; %u ; %i ; %u",lastCrsfTime, crsfTime ,offset,updated_interval);
        db_out.println("");
      }
      #endif
      crsfTime += updated_interval - offset;//set current micros
      lastCrsfTime = crsfTime; //set time that we send last packet
    #endif
    }
  }
}

void setup() {

  //analogReference(EXTERNAL);
  pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT);
  pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT);
  pinMode(DIGITAL_PIN_LED, OUTPUT);//LED
  //pinMode(DIGITAL_PIN_BUZZER, OUTPUT);//
  //digitalWrite(DIGITAL_PIN_BUZZER, LOW);
  //batteryVoltage=7.0; 
  
  xTaskCreatePinnedToCore(
                    OutputTask,   /* Task function. */
                    "OutputTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
                    delay(500); 

  xTaskCreatePinnedToCore(
                    ElrsTask,   /* Task function. */
                    "ElrsTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    -1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
                    delay(500); 
  
}
    
void loop() {
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
        //     Set Lua [BLE Joystick]=3 sending response for [BLE Joystick] chunk=0 step=3
        //     Set Lua [BLE Joystick]=4 to enable
        //                hwTimer stop
        //                Set Lua [TX Power]=0
        //                hwTimer interval: 5000
        //                Adjusted max packet size 22-22
        //                Starting BLE Joystick!
        // 19: Set Lua [Bad/Good]=0
        // 20: Set Lua [2.1.0 EU868]=0 =1 ?? get 


