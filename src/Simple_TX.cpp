
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

//#define DEBUG_PACKETS
//#define DEBUG_TLM
//#define DEBUG_CH
//#define DEBUG_SYNC
//#define DEBUG_HALF_DUPLEX


#include <Arduino.h>

#include "config.h"
#include "crsf.h"
#include "led.h"
#include "oled.h"
#include "crsf_protocol.h"
#include "halfduplex.h"
#include "uart.h"
#include "menus.h"
#include "ui_buttons.h"
#include "gpio.h"


Oled oled;

char tempstring[TEMPSTRINGLENGTH];

TaskHandle_t elrsTaskHandler;
TaskHandle_t outputTaskHandler;


//click deboucer
uint32_t clickCurrentMicros = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];


//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


void OutputTask( void * pvParameters ){
 
  oled.init();


  for(;;){
    read_ui_buttons();
     if ((MODULE_IS_ELRS)&&(local_info.good_pkts==0)) {
      CRSF_get_elrs(crsfCmdPacket);
      elrsWrite(crsfCmdPacket,6,0);
    } 
    
    if (entered == -1) {
      if (params_loaded<crsf_devices[0].number_of_params) {
        char *load = (char *)hdr_str_cb(menuItems);//TODO
        dbout.printf("hdr:%s\n",load);
        oled.printf(load);

        if (crsf_devices[device_idx].number_of_params) {
          if (crsf_devices[device_idx].address == ADDR_RADIO) {
            dbout.println("address:radio");
            //protocol_read_param(device_idx, &crsf_params[0]);    // only one param now
          } else {
            dbout.printf("Menu address: 0x%x - num par: %u\n",crsf_devices[device_idx].address, crsf_devices[device_idx].number_of_params);
            if (next_param > 0) {
              CRSF_read_param(crsfCmdPacket,next_param,0);
              elrsWrite(crsfCmdPacket,8,200000);
            }
          }
        }
        delay(2000);
      // end not all params
      } else { //else (if all parameters) load main menu      
          //dbout.println("main menu loaded...");
          oled.setMainMenuItems();
      }
    } else if (entered== -2) { //show idle screen
   // dbout.printf("idle screen\n");
      oled.setMainScreen(
              crsf_devices[0].name,
              LinkStatistics,
              local_info.bad_pkts,
              local_info.good_pkts);
    } //end idle screen
    else {
      oled.setSubMenuItems();
    }
  } // end main loop for
} // end output task
  
void bt_handle(uint8_t value) {
  dbout.println("bt_handle");
  
  powerChangeHasRun=true;   

  clickCurrentMicros = crsfTime + 500000;//0.5sec
  dbout.printf("times: %u:%u\n", clickCurrentMicros/1000, crsfTime/1000);
  //powerChangeHasRun=true;
  
  //CRSF_read_param(crsfCmdPacket,1,next_chunk);
  //elrsWrite(crsfCmdPacket,8,0);
  
  //buildElrsPingPacket(crsfCmdPacket);
  //dbout.println(CRSF_send_model_id(2));
  
  //set modelId
  //CRSF_sendId(crsfSetIdPacket,0);
  //elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength);

  //turn on rx wifi, even if missmatch modelId
  //buildElrsPacket(crsfCmdPacket,16,1);

  CRSF_read_param(crsfCmdPacket,1,next_chunk);
  elrsWrite(crsfCmdPacket,8,200000);
  //serialEvent();
}



static void crsfdevice_init() {
    next_param = 1;
    next_chunk = 0;
    recv_param_ptr = recv_param_buffer;
    params_loaded = 0;
    //CBUF_Init(send_buf);
}

//Task2 - ELRS task - main loop
void ElrsTask( void * pvParameters ){
  //setup channels
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
    rcChannels[i] = RC_CHANNEL_MIN;
  }
 
  //uart debug
  dbout.begin(115200);
  delay(2000); 
  elrs.begin(SERIAL_BAUDRATE,SERIAL_8N1,13, 13,false, 500);
  dbout.write("starting elrs\n");
  //digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON
  device_idx = 0;
  crsfdevice_init();
  //dbout.printf("********hdr : %s\n",(char *) hdr_str_cb(param));

  for(;;){
    uint32_t currentMicros = micros();

    //read values of rcChannels
    Aileron_value = analogRead(analogInPinAileron); 
    Elevator_value= analogRead(analogInPinElevator); 
    Throttle_value= analogRead(analogInPinThrottle); 
    Rudder_value = analogRead(analogInPinRudder);
    Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    FlightMode = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    //map rcchannels
    rcChannels[0] = map(Aileron_value,0,4095,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
    rcChannels[1] = map(Elevator_value,0,4095,RC_CHANNEL_MIN,RC_CHANNEL_MAX); 
    rcChannels[2] = map(Throttle_value,0,4095,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
    rcChannels[3] = map(Rudder_value ,0,4095,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
    //Aux 1 Arm Channel
    rcChannels[4] = Arm ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    //dbout.println(Arm)
    //Aux 2 Mode Channel
    rcChannels[5] = FlightMode ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
    //Additional switch add here.
    //rcChannels[6] = CH6 ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
      
      
    testButtonPressed = digitalRead(DigitalInPinPowerChange);
  
    if (currentMicros >= crsfTime) {
      //dbout.printf("loop: %u:%u:%i \n",crsfTime,updateInterval,correction);
      if (powerChangeHasRun==true && clickCurrentMicros < currentMicros) 
      {
        //dbout.println("reset");
        powerChangeHasRun = false;
        clickCurrentMicros = currentMicros;
      }
      if((testButtonPressed==0) && (powerChangeHasRun==false)){
          dbout.println("click");
          bt_handle(1);//TODO
      } else { //send channels packets
          #if defined(DEBUG_CH)
            char buf [64];
            sprintf (buf, "A:%i:%i;E:%i:%i;T:%i:%i;R:%i:%i;arm:%i;fm:%i\r\n", 
            Aileron_value,rcChannels[0],
            Elevator_value,rcChannels[1],
            Throttle_value,rcChannels[2],
            Rudder_value,rcChannels[3],
            Arm,FlightMode);//batteryVoltage);
            delay(1000); 
          #else
            //char buf [64];
       /*      dbout.printf(buf, "A:%i:%i;E:%i:%i;T:%i:%i;R:%i:%i;arm:%i;fm:%i\r\n", 
            Aileron_value,rcChannels[0],
            Elevator_value,rcChannels[1],
            Throttle_value,rcChannels[2],
            Rudder_value,rcChannels[3],
            Arm,FlightMode); */

            //send crsf packet
            crsfPreparePacket(crsfPacket, rcChannels);
            elrsWrite(crsfPacket, CRSF_PACKET_SIZE,0);
          #endif
      } // end elrs loop
      //start receiving at end of each crsf cycle or cmd sent

      serialEvent();

    } // end button filter to send commands 

  } //end main loop
}

void setup() {

  initGpio();

  xTaskCreatePinnedToCore(
    ElrsTask,               /* Task function. */
    "ElrsTask",             /* name of task. */
    10000,                  /* Stack size of task */
    NULL,        /* parameter of the task */
    -1,           /* priority of the task */
    &elrsTaskHandler,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
    delay(500); 

  xTaskCreatePinnedToCore(
  OutputTask,
  "OutputTask",
  10000,
  NULL,
  1,
  &outputTaskHandler,
  0);          /* pin task to core 0 */                  
  delay(500); 
}
    
void loop() {
}

////////////////////////
////////////////////////


static const char *hdr_str_cb(const void *data) {
    
    (void)data;
     //   dbout.printf("call params: %u: %i\n",count_params_loaded(), device_idx);

    if (count_params_loaded() != crsf_devices[device_idx].number_of_params) {
   //     dbout.printf("not all params: %u: %i\n",count_params_loaded(), device_idx);
    
        snprintf(tempstring, sizeof tempstring, "%s %s", crsf_devices[device_idx].name, "LOADING");
    
    } else if (protocol_module_is_elrs()) {
        dbout.printf("idx_elrs: %i\n",device_idx);

        snprintf(tempstring, sizeof tempstring, "%s  %d/%d  %c",
                 crsf_devices[device_idx].name, elrs_info.bad_pkts, elrs_info.good_pkts,
                 (elrs_info.flags & 1) ? 'C' : '-');
    } else  {
        dbout.printf("tx module not detected\n");
        //return crsf_devices[device_idx].name;
        snprintf(tempstring, sizeof tempstring, "%s  %d/%d  %c",
                 crsf_devices[device_idx].name, elrs_info.bad_pkts, elrs_info.good_pkts,
                 (elrs_info.flags & 1) ? 'C' : '-');
    }
    return tempstring;
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


