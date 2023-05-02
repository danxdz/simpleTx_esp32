
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

#define DEBUG_PACKETS
//#define DEBUG_TLM
//#define DEBUG_CH
//#define DEBUG_SYNC
//#define DEBUG_HALF_DUPLEX
//#define DEBUG_CRSF_FRAMETYPE_RADIO_ID

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

TaskHandle_t elrsTaskHandler;
TaskHandle_t outputTaskHandler;

rc_input_t rcInput;

UI_input_t UIinput;

//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void OutputTask(void *pvParameters)
{

  Oled oled;
  oled.init();

  for (;;)
  {
    //entered = -2;


    readUIbuttons (&UIinput);
    read_ui_buttons(&UIinput);

     if (entered == -1)
    { // main menu -1
      if (params_loaded < crsf_devices[0].number_of_params)
      {
        char *load = (char *)hdr_str_cb(menuItems); // TODO
        dbout.printf("hdr:%s\n", load);

        oled.PrintLoad(load);

        if (crsf_devices[0].number_of_params)
        {
          if (crsf_devices[0].address == ADDR_RADIO)
          {
            dbout.println("address:radio");
            // protocol_read_param(device_idx, &crsf_params[0]);    // only one param now
          }
          else
          {
            // dbout.printf("Menu address: 0x%x - num par: %u : next_p:%u\n",crsf_devices[0].address, crsf_devices[0].number_of_params,next_param);
            if (next_param > 0)
            {
              // next_chunk = 0;
              CRSF_read_param(next_param, 0, ELRS_ADDRESS);
            }
          }
        }
        // delay(2000);
        // end not all params
      }
      else
      { // else (if all parameters) load main menu
        // dbout.println("main menu loaded...");
        oled.setMainMenuItems();
      }
    }
    else if (entered == -2)
    { // show idle screen
      // dbout.printf("idle screen\n");
      oled.setMainScreen(
          crsf_devices[0].name,
          LinkStatistics,
          local_info.bad_pkts,
          local_info.good_pkts);
    } // end idle screen
    else if (entered <= -10)
    { // click on mainmenu item to select option
      // db_out.printf("main select option\n");
      Oled::selectOptionMainMenu();
    }
    else if (entered >= 0)
    {
      Oled::setSubMenuItems();
    }
     
  } // end main loop for
} // end output task

// Task2 - ELRS task - main loop
void ElrsTask(void *pvParameters)
{
  // setup channels
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++)
  {
    rcChannels[i] = RC_CHANNEL_MIN;
  }

  // uart debug
  dbout.begin(115200);
  delay(2000);
  elrs.begin(SERIAL_BAUDRATE, SERIAL_8N1, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, false, 500);
  dbout.write("starting elrs\n");
  // digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON
  device_idx = 0;
  crsfdevice_init();
  delay(2000);

  for (;;)
  {
    uint32_t currentMicros = micros();
    if (currentMicros > tickTime)
    {
      check_link_state(currentMicros);
    }


    if (currentMicros >= crsfTime)
    {
      // dbout.printf("loop: %u:%u:%i \n",crsfTime,updateInterval,correction);
      if (powerChangeHasRun == true && clickCurrentMicros < currentMicros)
      {
        // dbout.println("reset");
        powerChangeHasRun = false;
        clickCurrentMicros = currentMicros;
      }
 
      else
      {
        // read values of rcChannels
        gpioReadInputs(&rcInput);

        // TODO: channel mixer
        gpioMixer(&rcInput);

        // send crsf channels packet
        crsfSendChannels(&rcInput);
      } // end elrs loop
      // start receiving at end of each crsf cycle or cmd sent

      serialEvent();

    } // end button filter to send commands

  } // end main loop
}

void setup()
{

  initGpio();
  //initUsb();

  xTaskCreatePinnedToCore(
      ElrsTask,         /* Task function. */
      "ElrsTask",       /* name of task. */
      10000,            /* Stack size of task */
      NULL,             /* parameter of the task */
      -1,               /* priority of the task */
      &elrsTaskHandler, /* Task handle to keep track of created task */
      1);               /* pin task to core 1 */
  delay(500);

  xTaskCreatePinnedToCore(
      OutputTask,
      "OutputTask",
      10000,
      NULL,
      1,
      &outputTaskHandler,
      0); /* pin task to core 0 */
  delay(500);
}

void loop()
{
}

////////////////////////
////////////////////////

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

/*

0:Packet Rate:0:9:0:4:4;25Hz(-123dBm):50Hz(-120dBm):100Hz(-117dBm):100Hz Full(-112dBm):200Hz(-112dBm): :: OPT
1:Telem Ratio:0:9:0:9:0;Std:Off:1:128:1:64:1:32:1:16:1:8:1:4:1:2:Race: :: OPT
2:Switch Mode:0:9:0:1:0;Wide:Hybrid: :: OPT
3:Model Match:0:9:0:1:0;Off:On: :: OPT
4:TX Power (50mW):0:11:0:0:0 :: MainMenuItem 
5:Max Power:5:9:0:2:2;10:25:50: :: OPT
6:Dynamic:5:9:0:5:0;Off:Dyn:AUX9:AUX10:AUX11:AUX12: :: OPT
7:VTX Administrator:0:11:0:0:0 :: MainMenuItem
8:Band:8:9:0:6:0;Off:A:B:E:F:R:L: :: OPT
9:Channel:8:9:0:7:0;1:2:3:4:5:6:7:8: :: OPT
10:Pwr Lvl:8:9:0:8:0;-:1:2:3:4:5:6:7:8: :: OPT
11:Pitmode:8:9:0:8:79;Off:On:AUX1�:AUX1�:AUX2�:AUX2�:AUX3�:AUX3�:AUX  Pitmode: :: OPT
12:Send VTx:8:13:0:0:0 :: CMD
13:WiFi Connectivity:0:11:0:0:0 :: MainMenuItem
14:Enable WiFi:14:13:0:0:0 :: CMD
15:Enable Rx WiFi:14:13:0:0:0 :: CMD
16:BLE Joystick:0:13:0:0:0 :: CMD
17:Bind:0:13:0:0:0 :: CMD
18:Bad/Good:0:12:0:0:0 :: INFO
19:3.1.2 EU868:0:12:0:0:0 :: INFO

as menu:

Main Menu
|- Packet Rate (OPT)
   |- 25Hz(-123dBm)
   |- 50Hz(-120dBm)
   |- 100Hz(-117dBm)
   |- 100Hz Full(-112dBm)
   |- 200Hz(-112dBm)
|- Telem Ratio (OPT)
   |- Std
   |- Off
   |- 1:128
   |- 1:64
   |- 1:32
   |- 1:16
   |- 1:8
   |- 1:4
   |- 1:2
   |- Race
|- Switch Mode (OPT)
   |- Wide
   |- Hybrid
|- Model Match (OPT)
   |- Off
   |- On
|- TX Power (50mW)
|  |- Max Power (OPT)
   |  |- 10
   |  |- 25
   |  |- 50
   |- Dynamic (OPT)
   |  |- Off
   |  |- Dyn
   |  |- AUX9
   |  |- AUX10
   |  |- AUX11
   |  |- AUX12
|- VTX Administrator
|  |- Band (OPT)
   |  |- Off
   |  |- A
   |  |- B
   |  |- E
   |  |- F
   |  |- R
   |  |- L
   |- Channel (OPT)
   |  |- 1
   |  |- 2
   |  |- 3
   |  |- 4
   |  |- 5
   |  |- 6
   |  |- 7
   |  |- 8
   |- Pwr Lvl (OPT)
   |  |- -
   |  |- 1
   |  |- 2
   |  |- 3
   |  |- 4
   |  |- 5
   |  |- 6
   |  |- 7
   |  |- 8
   |- Pitmode (OPT)
      |- Off
      |- On
|- Send VTx
|- WiFi Connectivity
|  |- Enable WiFi
   |- Enable Rx WiFi
|- BLE Joystick
|- Bind
|- Bad/Good
|- 3.1.2 EU868


*/

