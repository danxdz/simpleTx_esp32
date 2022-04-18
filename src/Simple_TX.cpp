
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
#include <HardwareSerial.h>

#include "config.h"
#include "crsf.c"
#include "led.h"
#include "tlm.h"
#include "model.h"
#include "CrsfProtocol/crsf_protocol.h"
#include "halfduplex.h"
#include "uart.h"
#include "menus/menus.cpp"
#include "ui_buttons/ui_buttons.cpp"

char tempstring[TEMPSTRINGLENGTH];

TaskHandle_t elrsTaskHandler;
TaskHandle_t outputTaskHandler;

//elrs timing
uint32_t crsfTime = 0;
// for calculate main loop time to sync with elrs tx module
uint32_t lastCrsfTime = 0;
//click deboucer
uint32_t clickCurrentMicros = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];


static uint8_t SerialInBuffer[CRSF_MAX_PACKET_LEN];
//static u8 telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];

static uint32_t updateInterval = CRSF_TIME_BETWEEN_FRAMES_US;
static int32_t correction;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static uint8_t current_folder = 0;
static uint8_t params_displayed;  // if not zero, number displayed so far for current device
static uint8_t device_idx;   // current device index

static char recv_param_buffer[CRSF_MAX_CHUNKS * CRSF_MAX_CHUNK_SIZE];
static char *recv_param_ptr;
static struct crsfdevice_page * const mp = &pagemem.u.crsfdevice_page;

crsf_device_t radio = {
    .address = ADDR_RADIO,
    .number_of_params = 1,
    .params_version = 0,
    .serial_number = 1,
    .hardware_id = 0,
    .firmware_id = 0,
};


//TODO

elrs_info_t local_info;

elrs_info_t elrs_info;
crsf_device_t crsf_devices[CRSF_MAX_DEVICES];

crsf_param_t crsf_params[CRSF_MAX_PARAMS];
menu_items mItems[CRSF_MAX_PARAMS];
menu_items smItems[CRSF_MAX_PARAMS];

static uint8_t model_id_send;
static uint32_t elrs_info_time;
static module_type_t module_type;

#define MODULE_IS_ELRS     (module_type == MODULE_ELRS)
#define MODULE_IS_UNKNOWN  (module_type == MODULE_UNKNOWN)
void protocol_module_type(module_type_t type) {
    module_type = type;
};
uint8_t protocol_module_is_elrs() { return MODULE_IS_ELRS; }

void protocol_read_param(uint8_t device_idx, crsf_param_t *param) {
    // only protocol parameter is bitrate
    param->device = device_idx;            // device index of device parameter belongs to
    param->id = 1;                // Parameter number (starting from 1)
    param->parent = 0;            // Parent folder parameter number of the parent folder, 0 means root
    param->type = TEXT_SELECTION;  // (Parameter type definitions and hidden bit)
    param->hidden = 0;            // set if hidden
    param->name = (char*)crsf_opts[0];           // Null-terminated string
    param->value = (char *)&"400K\0001.87M\0002.25M";    // must match crsf_opts
    param->default_value = 0;  // size depending on data type. Not present for COMMAND.
    param->min_value = 0;        // not sent for string type
    param->max_value = 2;        // not sent for string type
    param->changed = 0;           // flag if set needed when edit element is de-selected
    param->max_str = &((char*)param->value)[11];        // Longest choice length for text select
    param->u.text_sel = Model.proto_opts[PROTO_OPTS_BITRATE];
}

static uint32_t parse_u32(const uint8_t *buffer) {
    return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
}
static void parse_device(uint8_t* buffer, crsf_device_t *device) {
    buffer += 2;
    device->address = (uint8_t) *buffer++;
    strlcpy(device->name, (const char *)buffer, CRSF_MAX_NAME_LEN);
    buffer += strlen((const char*)buffer) + 1;
    device->serial_number = parse_u32(buffer);
    buffer += 4;
    device->hardware_id = parse_u32(buffer);
    buffer += 4;
    device->firmware_id = parse_u32(buffer);
    buffer += 4;
    device->number_of_params = *buffer;
    buffer += 1;
    device->params_version = *buffer;
    if (device->address == ADDR_MODULE) {
      if (device->serial_number == 0x454C5253)
      {
        db_out.println("Module type: elrs");
        protocol_module_type(MODULE_ELRS);
      }
        else
      {
        db_out.println("Module type: not elrs");
        protocol_module_type(MODULE_OTHER);
      } 
    }
    db_out.printf("Module details:%s,0x%x,%u,%u,%u,%u,%u\n",
                    device->name,
                    device->address,
                    device->number_of_params,
                    device->params_version,
                    device->serial_number,
                    device->firmware_id,
                    device->hardware_id);
  db_out.printf("devices: %u : %u : 0x%x\n",device_idx, crsf_devices[device_idx].number_of_params,crsf_devices[device_idx].address);
  
}

static void add_device(uint8_t *buffer) {
            
    for (int i=0; i < CRSF_MAX_DEVICES; i++) {
        if (crsf_devices[i].address == buffer[2]        //  device already in table
         || crsf_devices[i].address == 0                //  not found, add to table
         || crsf_devices[i].address == ADDR_RADIO) {    //  replace deviation device if necessary
            db_out.printf("no devices match : parse new:0x%x\n",buffer[2]);

            parse_device(buffer, &crsf_devices[i]);
            break;
        }
    }
    //  no new device added if no more space in table
}






void CRSF_ping_devices() {
  buildElrsPingPacket(crsfCmdPacket);
  elrsWrite(crsfCmdPacket,6,0);
 }


static void parse_elrs_info(uint8_t *buffer) {
    local_info.bad_pkts = buffer[3];                      // bad packet rate (should be 0)
    local_info.good_pkts = (buffer[4] << 8) + buffer[5];  // good packet rate (configured rate)
  
    // flags bit 0 indicates receiver connected
    // other bits indicate errors - error text in flags_info
    local_info.flags = buffer[6];
    strlcpy(local_info.flag_info, (const char*)&buffer[7], CRSF_MAX_NAME_LEN);  // null-terminated text of flags
  
    local_info.update = elrs_info.update;
    if (memcmp((void*)&elrs_info, (void*)&local_info, sizeof(elrs_info_t)-CRSF_MAX_NAME_LEN)) {
        if (local_info.flag_info[0] && strncmp(local_info.flag_info, elrs_info.flag_info, CRSF_MAX_NAME_LEN)) {
            db_out.printf("error: %s\n",local_info.flag_info);
            //example: error: Model Mismatch
            //error: [ ! Armed ! ]
        }

        memcpy((void*)&elrs_info, (void*)&local_info, sizeof(elrs_info_t)-CRSF_MAX_NAME_LEN);
        elrs_info.update++;
   }
   
    //example bad_pckts : good_pckts ; flag ; flag_info ; info_update ;
    // 0 : 100 ; 5 ; Model Mismatch ; 0 
    // 0 : 200 ; 8 ; [ ! Armed ! ] ; 0 
    db_out.printf("%u : %u ; %u ; %s ; %u\n ",local_info.bad_pkts,local_info.good_pkts,local_info.flags,local_info.flag_info,local_info.update);
}

void CRSF_serial_rcv(uint8_t *buffer, uint8_t num_bytes) {
  /*for (int i=0;i<num_bytes;i++) {
    db_out.printf("0x%x:",buffer[i]);
    }
  db_out.println(""); */

  if ((buffer[0]!=CRSF_FRAMETYPE_RADIO_ID) && (buffer[0]!=CRSF_FRAMETYPE_LINK_STATISTICS)){
      //db_out.printf("CRSF FRAMETYPE: 0x%x : L:%u : ",buffer[0],num_bytes);
  } else {
     if (buffer[0]!=CRSF_FRAMETYPE_RADIO_ID) {
      for (int i=0;i<num_bytes;i++) {
        db_out.printf("0x%x:",buffer[i]);
      }
      db_out.println("");
    } 
  }
  switch (buffer[0]) {
    case CRSF_FRAMETYPE_DEVICE_INFO:
       db_out.printf("DEVICE_INFO\n");

        add_device(buffer);
        db_out.printf("send model id\n");

        CRSF_sendId(crsfSetIdPacket,0);
        elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength,0);
        break;

    case CRSF_FRAMETYPE_ELRS_STATUS:
        parse_elrs_info(buffer);
       // db_out.printf("FRAMETYPE_ELRS_STATUS\n");
    
        break;

    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
     //   db_out.printf("PARAMETER_SETTINGS_ENTRY\n");
        add_param(buffer, num_bytes);
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
  //set uart as rx
  duplex_set_RX();
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
          int32_t value;
          uint8_t id = SerialInBuffer[2];
          CRSF_serial_rcv(SerialInBuffer+2, SerialInBuffer[1]-1);


          if (id == CRSF_FRAMETYPE_BATTERY_SENSOR) {
            db_out.print("battery");
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
              if (MODULE_IS_UNKNOWN) {
                CRSF_ping_devices();
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
          
          #if defined(DEBUG_PACKETS)
          //output packets to serial for debug
            for (int i=0;i<=15;i++) {
              db_out.write(SerialInBuffer[i]); 
            }
          #endif
    
        
        } else {
          db_out.write("UART CRC failure");
          // cleanup input buffer
          elrs.flush();
          // BadPktsCount++;
        }
        CRSFframeActive = false;
        SerialInPacketPtr = 0;
        SerialInPacketLen = 0;
      }
    }  
  }
}



void OutputTask( void * pvParameters ){
  Oled::init();

  for(;;){
    read_ui_buttons();

     if ((MODULE_IS_ELRS)&&(local_info.good_pkts==0)) {
      CRSF_get_elrs(crsfCmdPacket);
      elrsWrite(crsfCmdPacket,sizeof(crsfCmdPacket),0);
    } 
    
    if (entered == -1) {
      if (params_loaded<crsf_devices[0].number_of_params) {
        char *load = (char *)hdr_str_cb(mItems);
        //db_out.printf("hdr:%s\n",load);
        Menu::loadMainMenu(load);
        if (crsf_devices[device_idx].number_of_params) {
          if (crsf_devices[device_idx].address == ADDR_RADIO) {
            //db_out.println("address:radio");
            protocol_read_param(device_idx, &crsf_params[0]);    // only one param now
          } else {
            //db_out.printf("Menu address: 0x%x - num par: %u\n",crsf_devices[device_idx].address, crsf_devices[device_idx].number_of_params);
            if (next_param ==1) {
              CRSF_read_param(crsfCmdPacket,next_param,0);
              elrsWrite(crsfCmdPacket,8,200000);
            }
          }
        }
        delay(2000);
      // end not all params
      } else { //else (if all parameters) load main menu      
        //db_out.println("main menu loaded...");
        Oled::setMainMenuItems();
      }
    } else if (entered== -2) { //show idle screen
      Oled::setMainScreen(
              crsf_devices[0].name,
              LinkStatistics,
              local_info.bad_pkts,
              local_info.good_pkts);
    } //end idle screen
    else {
      Oled::setSubMenuItems();
    }
  } // end main loop for
} // end output task
  
void bt_handle(uint8_t value) {
  db_out.println("bt_handle");
  
  powerChangeHasRun=true;   
  clickCurrentMicros = crsfTime + 500000;//0.5sec
  db_out.printf("times: %u:%u\n", clickCurrentMicros/1000, crsfTime/1000);
  //powerChangeHasRun=true;
  
  //CRSF_read_param(crsfCmdPacket,1,next_chunk);
  //elrsWrite(crsfCmdPacket,8,0);
  
  //buildElrsPingPacket(crsfCmdPacket);
  //db_out.println(CRSF_send_model_id(2));
  
  //set modelId
  //CRSF_sendId(crsfSetIdPacket,0);
  //elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength);

  //turn on rx wifi, even if missmatch modelId
  //buildElrsPacket(crsfCmdPacket,16,1);

  CRSF_read_param(crsfCmdPacket,1,next_chunk);
  elrsWrite(crsfCmdPacket,8,200000);
  //serialEvent();
}

void sync_crsf (int32_t add_delay) {
  crsfTime = micros();//set current micros
  int32_t offset = (crsfTime-lastCrsfTime);//get dif between pckt send
  uint32_t updated_interval = get_update_interval();
  //debug timing
  #if defined(DEBUG_SYNC)
    if (updated_interval!=20000)
      db_out.printf("%u ; %u ; %i ; %u\n",lastCrsfTime, crsfTime ,offset,updated_interval);
  #endif
  //if (add_delay>0)
    //db_out.printf("delay:%u:%u\n",add_delay,offset);
  crsfTime += ((updated_interval+add_delay) - offset);//set current micros
  lastCrsfTime = crsfTime; //set time that we send last packet
}

static void crsfdevice_init() {
    next_param = 1;
    next_chunk = 0;
    recv_param_ptr = recv_param_buffer;
    params_loaded = 0;
    params_displayed = 0;
    next_string = mp->strings;
    memset(mItems, 0, sizeof mItems);
    //CBUF_Init(send_buf);
}

//Task2 - ELRS task - main loop
void ElrsTask( void * pvParameters ){
  //setup channels
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
    rcChannels[i] = RC_CHANNEL_MIN;
  }
 
  //uart debug
  db_out.begin(115200);
  delay(2000); 
  elrs.begin(SERIAL_BAUDRATE,SERIAL_8N1,13, 13,false, 500);
  db_out.write("starting elrs\n");
  //digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON
  device_idx = 0;
  crsfdevice_init();
  current_folder = 0;
  crsf_param_t *param;
  db_out.printf("********hdr : %s\n",(char *) hdr_str_cb(param));

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
    //db_out.println(Arm)
    //Aux 2 Mode Channel
    rcChannels[5] = FlightMode ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
    //Additional switch add here.
    //rcChannels[6] = CH6 ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
      
      
    testButtonPressed = digitalRead(DigitalInPinPowerChange);
  
    if (currentMicros >= crsfTime) {
        //db_out.printf("loop: %i %i :%u:%u \n",
      if (powerChangeHasRun==true && clickCurrentMicros < currentMicros) 
      {
        //db_out.println("reset");
        powerChangeHasRun = false;
        clickCurrentMicros = currentMicros;
      }
      if((testButtonPressed==0) && (powerChangeHasRun==false)){
          db_out.println("click");
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
       /*      db_out.printf(buf, "A:%i:%i;E:%i:%i;T:%i:%i;R:%i:%i;arm:%i;fm:%i\r\n", 
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

////////////////////////////////////////////////////////


static void  elrsWrite (
  uint8_t crsfPacket[],uint8_t size,int32_t add_delay) {
  duplex_set_TX();
  elrs.write(crsfPacket, size);
  elrs.flush();
  //if (add_delay>0)
    //db_out.printf("del:%u\n",add_delay);

  //set last time packet send
  sync_crsf(add_delay);
}

#define MIN(a, b) ((a) < (b) ? a : b)

static uint8_t count_params_loaded() {
    int i;
    for (i=0; i < crsf_devices[0].number_of_params; i++) {
        //db_out.printf("count_params_loaded: %i:%u\n",i,crsf_devices[0].number_of_params);
        if (menuItems[i].id == 0) break;
    }
    return i;
}

static void add_param(uint8_t *buffer, uint8_t num_bytes) {
  // abort if wrong device, or not enough buffer space
  db_out.printf("add_param:%u:%u:0x%x\n",buffer[3],crsf_devices[0].number_of_params,crsf_devices[device_idx].address);
  //CRSF_ADDRESS_CRSF_TRANSMITTER 
  if (buffer[2] != crsf_devices[device_idx].address
       || ((int)((sizeof recv_param_buffer) - (recv_param_ptr - recv_param_buffer)) < (num_bytes-4))) {
        recv_param_ptr = recv_param_buffer;
        next_chunk = 0;
        next_param = 0;
        db_out.println("err");
        db_out.println(recv_param_ptr);
        return;
    }
    memcpy(recv_param_ptr, buffer+5, num_bytes-5);
    recv_param_ptr += num_bytes - 5;

    if (buffer[4] > 0) {
        if (buffer[4] >= CRSF_MAX_CHUNKS) {
            recv_param_ptr = recv_param_buffer;
            next_chunk = 0;
            next_param = 0;
            return;
        } else {
            next_chunk += 1;
            CRSF_read_param(crsfCmdPacket,next_param, next_chunk);
            elrsWrite(crsfCmdPacket,8,20000);

        }
        return;
    }

    // received all chunks for current parameter
    recv_param_ptr = recv_param_buffer;
    // all devices so far start parameter id at 1...fingers crossed
    if (buffer[3] >= CRSF_MAX_PARAMS) {
        next_chunk = 0;
        next_param = 0;
        return;
    }
    menuItems[(int)buffer[3]-1].getParams(recv_param_ptr,buffer[3]);
    //debug
    //menuItems[buffer[3]-1].displayInfo();


    recv_param_ptr = recv_param_buffer;
    next_chunk = 0;
    params_loaded = count_params_loaded();
    // read all params when needed
   /// db_out.printf("pL:%u:%i:%i\n",params_loaded,menu_item_id,submenu_item_id);
    if (params_loaded < crsf_devices[device_idx].number_of_params)
       // && (menu_item_id+submenu_item_id <  crsf_devices[device_idx].number_of_params))
    {
      if (next_param < crsf_devices[device_idx].number_of_params)
      {
        next_param += 1;
      } else {
        next_param = 1;
      }
       // db_out.printf("count_out:%u:%u:%u\n",
     // device_idx,
      //  crsf_devices[device_idx].number_of_params,params_loaded);
      //db_out.printf("id:%u:%s\n",parameter->id,parameter->name);

        CRSF_read_param(crsfCmdPacket, next_param, next_chunk);
        elrsWrite(crsfCmdPacket,8,0); 
  } else {
    /*  db_out.printf("0_count_out:%u:%u:%u\n",
        device_idx,
        crsf_devices[device_idx].number_of_params,
        params_loaded); */
       
    next_param = 0; 
    }
}


static const char *hdr_str_cb(const void *data) {
    
    (void)data;
     //   db_out.printf("call params: %u: %i\n",count_params_loaded(), device_idx);

    if (count_params_loaded() != crsf_devices[device_idx].number_of_params) {
   //     db_out.printf("not all params: %u: %i\n",count_params_loaded(), device_idx);
    
        snprintf(tempstring, sizeof tempstring, "%s %s", crsf_devices[device_idx].name, "LOADING");
    
    } else if (protocol_module_is_elrs()) {
        db_out.printf("idx_elrs: %i\n",device_idx);

        snprintf(tempstring, sizeof tempstring, "%s  %d/%d  %c",
                 crsf_devices[device_idx].name, elrs_info.bad_pkts, elrs_info.good_pkts,
                 (elrs_info.flags & 1) ? 'C' : '-');
    } else  {
        db_out.printf("tx module \n not detected: %i\n");
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


