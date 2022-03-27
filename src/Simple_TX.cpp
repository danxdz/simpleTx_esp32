
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
//#include "battery.h"
#include "tlm.h"
#include "gpio/gpio.cpp"

#include "CrsfProtocol/crsf_protocol.h"

#include "oled.h"
#include "halfduplex.h"
#include "uart.h"

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

static int32_t read_timeout;
static uint8_t current_folder = 0;
static uint8_t params_loaded;     // if not zero, number received so far for current device
static uint8_t params_displayed;  // if not zero, number displayed so far for current device
static uint8_t device_idx;   // current device index
static uint8_t next_param;   // parameter and chunk currently being read
static uint8_t next_chunk;
char recv_param_buffer[CRSF_MAX_CHUNKS * CRSF_MAX_CHUNK_SIZE];
char *recv_param_ptr;
char Modelname[24];

crsf_device_t deviation = {
    .address = ADDR_RADIO,
    .number_of_params = 1,
    .params_version = 0,
    .serial_number = 1,
    .hardware_id = 0,
    .firmware_id = 0,
};


elrs_info_t local_info;

elrs_info_t elrs_info;
crsf_device_t crsf_devices[CRSF_MAX_DEVICES];



static uint8_t model_id_send;
static uint32_t elrs_info_time;
static module_type_t module_type;

#define MODULE_IS_ELRS     (module_type == MODULE_ELRS)
#define MODULE_IS_UNKNOWN  (module_type == MODULE_UNKNOWN)
void protocol_module_type(module_type_t type) {
    module_type = type;
};
uint8_t protocol_module_is_elrs() { return MODULE_IS_ELRS; }





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
        db_out.println("elrs");
        protocol_module_type(MODULE_ELRS);
      }
        else
      {
        db_out.println("not elrs");
        protocol_module_type(MODULE_OTHER);
      } 
    }
    db_out.println(device->name);
    db_out.println(device->address);
    db_out.println(device->number_of_params);
    db_out.println(device->params_version);
    db_out.println(device->serial_number);
    db_out.println(device->firmware_id);
    db_out.println(device->hardware_id);
}
static void add_device(uint8_t *buffer) {
    for (int i=0; i < CRSF_MAX_DEVICES; i++) {
        if (crsf_devices[i].address == buffer[2]        //  device already in table
         || crsf_devices[i].address == 0                //  not found, add to table
         || crsf_devices[i].address == ADDR_RADIO) {    //  replace deviation device if necessary
            parse_device(buffer, &crsf_devices[i]);
            break;
        }
    }
    //  no new device added if no more space in table
}




void debug_output_uart(char *msg) {
      db_out.print(msg);
}

void CRSF_ping_devices() {
  buildElrsPingPacket(crsfCmdPacket);
  duplex_set_TX();
  elrsWrite(crsfCmdPacket,CRSF_CMD_PACKET_SIZE);
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
            db_out.printf("error: %s",local_info.flag_info);
            //error: Model Mismatch
            db_out.println("");
        }

        memcpy((void*)&elrs_info, (void*)&local_info, sizeof(elrs_info_t)-CRSF_MAX_NAME_LEN);
        elrs_info.update++;
   }
   
   //0 : 100 ; 5 ; Model Mismatch ; 0 
    db_out.printf("%u : %u ; %u ; %s ; %u ",local_info.bad_pkts,local_info.good_pkts,local_info.flags,local_info.flag_info,local_info.update);
    db_out.println("");
}

void CRSF_serial_rcv(uint8_t *buffer, uint8_t num_bytes) {
    if ((buffer[0]!=CRSF_FRAMETYPE_RADIO_ID) && (buffer[0]!=CRSF_FRAMETYPE_LINK_STATISTICS)){
      //db_out.printf("%u",buffer[0]);
      //db_out.println("");
    }
    switch (buffer[0]) {
    case CRSF_FRAMETYPE_DEVICE_INFO:
        db_out.printf("1: %u ; %u",buffer[0],num_bytes);
        db_out.println("");
        add_device(buffer);
        CRSF_sendId(crsfSetIdPacket,0);
        elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength);
        break;

    case CRSF_FRAMETYPE_ELRS_STATUS:
        parse_elrs_info(buffer);
        db_out.printf("2: %u ; %u",buffer[0],num_bytes);
        db_out.println("");
    
        break;

    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
//        db_out.printf("3: %u ; %u",buffer[0],num_bytes);
  //      db_out.println("");
    
        read_timeout = 0;
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
              if (MODULE_IS_UNKNOWN) CRSF_ping_devices();

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
    if ((MODULE_IS_ELRS)&&(local_info.good_pkts==0)) {
      CRSF_get_elrs(crsfCmdPacket);
      elrsWrite(crsfCmdPacket,sizeof(crsfCmdPacket));
    }
   
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
        local_info.bad_pkts,
        local_info.good_pkts,
        crsf_devices->name,
        module_type);
  } 
}

int powerButtonPressed=0;
bool powerChangeHasRun = false;
//TODO
// change to get last rate
uint8_t packetRateSelected = 0;
  
void bt_handle(uint8_t value) {
        if(packetRateSelected>=6) {
            packetRateSelected = 0;
        } else {
            packetRateSelected++;
        }
    //buildElrsPacket(crsfCmdPacket,value,packetRateSelected);
    CRSF_read_param(crsfCmdPacket,0,0);
    elrsWrite(crsfCmdPacket,CRSF_CMD_PACKET_SIZE);

    //buildElrsPingPacket(crsfCmdPacket);
    //db_out.println(CRSF_send_model_id(2));
    
    //set modelId
    //CRSF_sendId(crsfSetIdPacket,0);
    //elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength);

    //turn on rx wifi, even if missmatch modelId
    //buildElrsPacket(crsfCmdPacket,16,1);
    
    

    //serialEvent();
    powerChangeHasRun=true;
    clickCurrentMicros = crsfTime + (2*1000000);//2sec

    }

void sync_crsf () {
  crsfTime = micros();//set current micros
  int32_t offset = (crsfTime-lastCrsfTime);//get dif between pckt send
  uint32_t updated_interval = get_update_interval();
  //debug timing
  #if defined(DEBUG_SYNC)
    db_out.printf("%u ; %u ; %i ; %u",lastCrsfTime, crsfTime ,offset,updated_interval);
    db_out.println("");
  #endif
  crsfTime += updated_interval - offset;//set current micros
  lastCrsfTime = crsfTime; //set time that we send last packet
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
  //digitalWrite(DIGITAL_PIN_LED, LOW); //LED ON
    next_param = 1;
    next_chunk = 0;
    recv_param_ptr = recv_param_buffer;
    params_loaded = 0;
    params_displayed = 0;
    //next_string = mp->strings;
    memset(crsf_params, 0, sizeof crsf_params);

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
    rcChannels[4] = Arm ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;
    //db_out.println(Arm)
    //Aux 2 Mode Channel
    rcChannels[5] = FlightMode ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
    //Additional switch add here.
    //rcChannels[6] = CH6 ? RC_CHANNEL_MIN : RC_CHANNEL_MAX;   
    
    if (currentMicros >= crsfTime) {
      if (powerChangeHasRun==true && clickCurrentMicros < crsfTime) 
      {
        powerChangeHasRun = false;
        clickCurrentMicros = currentMicros;
        //CRSF_ping_devices();
      }
      powerButtonPressed = digitalRead(DigitalInPinPowerChange);
      if((powerButtonPressed==0) && (powerChangeHasRun==false)){
          db_out.println("click");
          bt_handle(1);//TODO
      } else {
          crsfPreparePacket(crsfPacket, rcChannels);
          #if defined(DEBUG_CH)
            char buf [64];
            sprintf (buf, "A:%i:%i;E:%i:%i;T:%i:%i;R:%i:%i;arm:%i;fm:%i\r\n", 
            Aileron_value,rcChannels[0],
            Elevator_value,rcChannels[1],
            Throttle_value,rcChannels[2],
            Rudder_value,rcChannels[3],
            Arm,FlightMode);//batteryVoltage);
            debug_output_uart(buf);
            delay(1000); 
          #else
            //send crsf packet
            elrsWrite(crsfPacket, CRSF_PACKET_SIZE);
          #endif
      }
      //start receiving at end of each crsf cycle
      serialEvent();
    }
  }
}

void setup() {

  initGpio();

  xTaskCreatePinnedToCore(
  OutputTask,
  "OutputTask",
  10000,
  NULL,
  1,
  &outputTaskHandler,
  0);          /* pin task to core 0 */                  
  delay(500); 

  xTaskCreatePinnedToCore(
    ElrsTask,               /* Task function. */
    "ElrsTask",             /* name of task. */
    10000,                  /* Stack size of task */
    NULL,        /* parameter of the task */
    -1,           /* priority of the task */
    &elrsTaskHandler,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
    delay(500); 
}
    
void loop() {
}
static void  elrsWrite (uint8_t crsfPacket[],uint8_t size) {
  duplex_set_TX();
  elrs.write(crsfPacket, size);
  elrs.flush();
  //set last time packet send
  sync_crsf();
}

static char *next_string;

#define MIN(a, b) ((a) < (b) ? a : b)

static void parse_bytes(enum data_type type, char **buffer, void *dest) {
    switch (type) {
    case UINT8:
        *(uint8_t *)dest = (uint8_t) (*buffer)[0];
        *buffer += 1;
        break;
    case INT8:
        *(int8_t *)dest = (int8_t) (*buffer)[0];
        *buffer += 1;
        break;
    case UINT16:
        *(uint16_t *)dest = (uint16_t) (((*buffer)[0] << 8) | (*buffer)[1]);
        *buffer += 2;
        break;
    case INT16:
        *(int16_t *)dest = (int16_t) (((*buffer)[0] << 8) | (*buffer)[1]);
        *buffer += 2;
        break;
    case FLOAT:
        *(int32_t *)dest = (int32_t) (((*buffer)[0] << 24) | ((*buffer)[1] << 16)
                     |        ((*buffer)[2] << 8)  |  (*buffer)[3]);
        *buffer += 4;
        break;
    default:
        break;
    }
}

static char *alloc_string(int32_t bytes) {
  if (CRSF_STRING_BYTES_AVAIL(next_string) < bytes)
        return NULL;

    char *p = next_string;
    next_string += bytes;
    return p;
}
static uint8_t count_params_loaded() {
    int i;
    for (i=0; i < CRSF_MAX_PARAMS; i++)
        if (crsf_params[i].id == 0) break;
    return i;
}


static void add_param(uint8_t *buffer, uint8_t num_bytes) {
  // abort if wrong device, or not enough buffer space
  //db_out.print("buffer: ");
  //db_out.printf("%u;%u;%u::%u - ",recv_param_ptr,recv_param_buffer,buffer[2],crsf_devices[0].address);
  int ts = ((bool)(((sizeof recv_param_buffer) - (recv_param_ptr - recv_param_buffer)) < (num_bytes-4)));
  //db_out.printf(" :%i:%u",ts,num_bytes);

  if (buffer[2] != CRSF_ADDRESS_CRSF_TRANSMITTER
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
            CRSF_read_param(crsfCmdPacket, next_param, next_chunk);
            elrsWrite(crsfCmdPacket,CRSF_CMD_PACKET_SIZE);

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
    crsf_param_t *parameter = &crsf_params[buffer[3]-1];
    int update = parameter->id == buffer[3];
    parameter->device = device_idx;
    parameter->id = buffer[3];
    parameter->parent = *recv_param_ptr++;

    parameter->type = static_cast<data_type>(*recv_param_ptr & 0x7f);
 
  /* for (int i=0;i<num_bytes;i++) {
    db_out.printf("\\x%02x",buffer[i]);
  } 
   */


  if (!update) {

        parameter->hidden = *recv_param_ptr++ & 0x80;
     //   db_out.printf("update: %u\n",parameter->hidden);
        
        parameter->name = (char *)recv_param_ptr,
                CRSF_STRING_BYTES_AVAIL(strlen(recv_param_ptr)+1);
    //    db_out.printf("name: %u:%s\n",recv_param_ptr,parameter->name);
        //recv_param_ptr += strlen(recv_param_ptr) + 1;
    } else {
        db_out.println("not update");

        if (parameter->hidden != (*recv_param_ptr & 0x80))
            params_loaded = 0;   // if item becomes hidden others may also, so reload all params
        parameter->hidden = *recv_param_ptr++ & 0x80;
        recv_param_ptr += strlen(recv_param_ptr) + 1;
    }
//    db_out.println("count");
    int count;
    switch (parameter->type) {
    case UINT8:
    case INT8:
    case UINT16:
    case INT16:
    case FLOAT:
        parse_bytes(parameter->type, &recv_param_ptr, &parameter->value);
        parse_bytes(parameter->type, &recv_param_ptr, &parameter->min_value);
        parse_bytes(parameter->type, &recv_param_ptr, &parameter->max_value);
        parse_bytes(parameter->type, &recv_param_ptr, &parameter->default_value);
        if (parameter->type == FLOAT) {
            parse_bytes(UINT8, &recv_param_ptr, &parameter->u.point);
            parse_bytes(FLOAT, &recv_param_ptr, &parameter->step);
        } else if (*recv_param_ptr) {
          db_out.println("eeerrr");
            if (!update) parameter->s.unit = ( char *)recv_param_ptr,
                    CRSF_STRING_BYTES_AVAIL(strlen(recv_param_ptr)+1);
        }
        break;

    case TEXT_SELECTION:
        if (!update) {
      //    db_out.println("text_sel");
            parameter->value = ( char *)recv_param_ptr,
                    CRSF_STRING_BYTES_AVAIL(strlen(recv_param_ptr)+1);
            recv_param_ptr += strlen(recv_param_ptr) + 1;
            // put null between selection options
            // find max choice string length to adjust textselectplate size
            char *start = (char *)parameter->value;
            int max_len = 0;
            count = 0;
            for (char *p = (char *)parameter->value; *p; p++) {
                if (*p == ';') {
                    *p = '\0';
                    if (p - start > max_len) {
                        parameter->max_str = start;  // save max to determine gui box size
                        max_len = p - start;
                    }
                    start = p+1;
                    count += 1;
                }
            }
            parameter->max_value = count;   // bug fix for incorrect max from device
           // db_out.printf("value:%s par_max: %i",parameter->value,parameter->max_value);
        } else {
            recv_param_ptr += strlen(recv_param_ptr) + 1;
        }
        parse_bytes(UINT8, &recv_param_ptr, &parameter->u.text_sel);
        parse_bytes(UINT8, &recv_param_ptr, &parameter->min_value);
        parse_bytes(UINT8, &recv_param_ptr, &count);  // don't use incorrect parameter->max_value
        parse_bytes(UINT8, &recv_param_ptr, &parameter->default_value);
        break;

    case INFO:
        if (!update) {
            parameter->value = (char *)recv_param_ptr,CRSF_STRING_BYTES_AVAIL(strlen(recv_param_ptr)+1);
            recv_param_ptr += strlen(recv_param_ptr) + 1;
        }
        break;

    case STRING:
        {
            const char *value, *default_value;
            value = recv_param_ptr;
            recv_param_ptr += strlen(value) + 1;
            default_value = recv_param_ptr;
            recv_param_ptr += strlen(default_value) + 1;
            parse_bytes(UINT8, &recv_param_ptr, &parameter->u.string_max_len);

            // No string re-sizing so allocate max length for value
            if (!update) 
              parameter->value = 
                (char *)MIN(parameter->u.string_max_len+1,
                CRSF_STRING_BYTES_AVAIL(parameter->u.string_max_len+1));
            if (!update) 
              parameter->default_value = 
                (char *) default_value,
                CRSF_STRING_BYTES_AVAIL(strlen(default_value)+1);
        }
        break;

    case COMMAND:
        parse_bytes(UINT8, &recv_param_ptr, &parameter->u.status);
        parse_bytes(UINT8, &recv_param_ptr, &parameter->timeout);
        if (!update) parameter->s.info = ( char *)recv_param_ptr, 20;

        command.param = parameter;
        command.time = 0;
        switch (parameter->u.status) {
        case PROGRESS:
            db_out.println("progress case");
            //command.time = CLOCK_getms();
            // FALLTHROUGH
        case CONFIRMATION_NEEDED:
            command.dialog = 1;
            break;
        case READY:
            command.dialog = 2;
            break;
        }
        break;

    case FOLDER:
    case OUT_OF_RANGE:
    default:
        break;
    }


  db_out.printf("%s:%u\n",
  //:%u:%u:%u:%u:%s:%s:%u:%i:%i:%u:%i:%s:%u",
  parameter->name,
  parameter->id
  );
  /* ,
  parameter->type,
  parameter->changed,
  parameter->device,
  parameter->max_str,
  parameter->default_value,
  parameter->hidden,
  parameter->max_value,
  parameter->min_value,
  parameter->parent,
  parameter->step,
  parameter->value,
  parameter->timeout);
 */
    recv_param_ptr = recv_param_buffer;
    next_chunk = 0;
    params_loaded = count_params_loaded();


    if (!update) {
        parameter->hidden = *recv_param_ptr++ & 0x80;
    recv_param_ptr = recv_param_buffer;
    next_chunk = 0;
    params_loaded = count_params_loaded();

    // read all params when needed
    if (params_loaded < crsf_devices[device_idx].number_of_params) {
        if (next_param < crsf_devices[device_idx].number_of_params)
            next_param += 1;
        else
            next_param = 1;
            CRSF_read_param(crsfCmdPacket, next_param, next_chunk);
            elrsWrite(crsfCmdPacket,CRSF_CMD_PACKET_SIZE);
    } else {
        next_param = 0;
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
        //     Set Lua [BLE Joystick]=3 sending response for [BLE Joystick] chunk=0 step=3
        //     Set Lua [BLE Joystick]=4 to enable
        //                hwTimer stop
        //                Set Lua [TX Power]=0
        //                hwTimer interval: 5000
        //                Adjusted max packet size 22-22
        //                Starting BLE Joystick!
        // 19: Set Lua [Bad/Good]=0
        // 20: Set Lua [2.1.0 EU868]=0 =1 ?? get 


