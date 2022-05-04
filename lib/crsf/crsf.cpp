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
 * along with Simple TX.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 =======================================================================================================
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * ELRS uses crossfire protocol at many different baud rates supported by EdgeTX i.e. 115k, 400k, 921k, 1.87M, 3.75M
 * 115000 bit/s = 14400 byte/s 
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */
#include "crsf.h"
#include <stdint.h>

#include "halfduplex.h"
#include "menus.h"
#include "uart.h"
#include "crsf_protocol.h"

uint32_t crsfTime = 0;
uint32_t lastCrsfTime = 0;
uint32_t updateInterval = CRSF_TIME_BETWEEN_FRAMES_US;;

int32_t correction = 0;

crsf_device_t crsf_devices[CRSF_MAX_DEVICES];

elrs_info_t  local_info;

elrs_info_t  elrs_info;

void protocol_module_type(module_type_t type) {
    module_type = type;
};
uint8_t protocol_module_is_elrs() { return MODULE_IS_ELRS; }


char recv_param_buffer[CRSF_MAX_CHUNKS * CRSF_MAX_CHUNK_SIZE];
char *recv_param_ptr;

uint8_t device_idx = 0;   // current device index

uint8_t SerialInBuffer[CRSF_MAX_PACKET_LEN];

//prepare elrs setup packet (power, packet rate...)
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
uint8_t crsfSetIdPacket[LinkStatisticsFrameLength];


 // crc implementation from CRSF protocol document rev7
static uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

// CRC8 implementation with polynom = 0xBA
static const uint8_t crc8tab_BA[256] = {
    0x00, 0xBA, 0xCE, 0x74, 0x26, 0x9C, 0xE8, 0x52, 0x4C, 0xF6, 0x82, 0x38, 0x6A, 0xD0, 0xA4, 0x1E,
    0x98, 0x22, 0x56, 0xEC, 0xBE, 0x04, 0x70, 0xCA, 0xD4, 0x6E, 0x1A, 0xA0, 0xF2, 0x48, 0x3C, 0x86,
    0x8A, 0x30, 0x44, 0xFE, 0xAC, 0x16, 0x62, 0xD8, 0xC6, 0x7C, 0x08, 0xB2, 0xE0, 0x5A, 0x2E, 0x94,
    0x12, 0xA8, 0xDC, 0x66, 0x34, 0x8E, 0xFA, 0x40, 0x5E, 0xE4, 0x90, 0x2A, 0x78, 0xC2, 0xB6, 0x0C,
    0xAE, 0x14, 0x60, 0xDA, 0x88, 0x32, 0x46, 0xFC, 0xE2, 0x58, 0x2C, 0x96, 0xC4, 0x7E, 0x0A, 0xB0,
    0x36, 0x8C, 0xF8, 0x42, 0x10, 0xAA, 0xDE, 0x64, 0x7A, 0xC0, 0xB4, 0x0E, 0x5C, 0xE6, 0x92, 0x28,
    0x24, 0x9E, 0xEA, 0x50, 0x02, 0xB8, 0xCC, 0x76, 0x68, 0xD2, 0xA6, 0x1C, 0x4E, 0xF4, 0x80, 0x3A,
    0xBC, 0x06, 0x72, 0xC8, 0x9A, 0x20, 0x54, 0xEE, 0xF0, 0x4A, 0x3E, 0x84, 0xD6, 0x6C, 0x18, 0xA2,
    0xE6, 0x5C, 0x28, 0x92, 0xC0, 0x7A, 0x0E, 0xB4, 0xAA, 0x10, 0x64, 0xDE, 0x8C, 0x36, 0x42, 0xF8,
    0x7E, 0xC4, 0xB0, 0x0A, 0x58, 0xE2, 0x96, 0x2C, 0x32, 0x88, 0xFC, 0x46, 0x14, 0xAE, 0xDA, 0x60,
    0x6C, 0xD6, 0xA2, 0x18, 0x4A, 0xF0, 0x84, 0x3E, 0x20, 0x9A, 0xEE, 0x54, 0x06, 0xBC, 0xC8, 0x72,
    0xF4, 0x4E, 0x3A, 0x80, 0xD2, 0x68, 0x1C, 0xA6, 0xB8, 0x02, 0x76, 0xCC, 0x9E, 0x24, 0x50, 0xEA,
    0x48, 0xF2, 0x86, 0x3C, 0x6E, 0xD4, 0xA0, 0x1A, 0x04, 0xBE, 0xCA, 0x70, 0x22, 0x98, 0xEC, 0x56,
    0xD0, 0x6A, 0x1E, 0xA4, 0xF6, 0x4C, 0x38, 0x82, 0x9C, 0x26, 0x52, 0xE8, 0xBA, 0x00, 0x74, 0xCE,
    0xC2, 0x78, 0x0C, 0xB6, 0xE4, 0x5E, 0x2A, 0x90, 0x8E, 0x34, 0x40, 0xFA, 0xA8, 0x12, 0x66, 0xDC,
    0x5A, 0xE0, 0x94, 0x2E, 0x7C, 0xC6, 0xB2, 0x08, 0x16, 0xAC, 0xD8, 0x62, 0x30, 0x8A, 0xFE, 0x44};

static uint8_t crsf_crc(const uint8_t crctab[], const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i=0; i < len; i++) {
        crc = crctab[crc ^ *ptr++];
    }
    return crc;
}
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    return crsf_crc(crc8tab, ptr, len);
}
uint8_t crsf_crc8_BA(const uint8_t *ptr, uint8_t len) {
    return crsf_crc(crc8tab_BA, ptr, len);
}

//prepare data packet
void crsfPreparePacket(uint8_t packet[], int channels[]){

    // packet[0] = UART_SYNC; //Header
    packet[0] = ADDR_MODULE; //Header
    packet[1] = 24;   // length of type (24) + payload + crc
    packet[2] = TYPE_CHANNELS;
    packet[3] = (uint8_t) (channels[0] & 0x07FF);
    packet[4] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
    packet[5] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
    packet[6] = (uint8_t) ((channels[2] & 0x07FF)>>2);
    packet[7] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
    packet[8] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
    packet[9] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
    packet[10] = (uint8_t) ((channels[5] & 0x07FF)>>1);
    packet[11] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
    packet[12] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
    packet[13] = (uint8_t) ((channels[7] & 0x07FF)>>3);
    packet[14] = (uint8_t) ((channels[8] & 0x07FF));
    packet[15] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
    packet[16] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);  
    packet[17] = (uint8_t) ((channels[10] & 0x07FF)>>2);
    packet[18] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
    packet[19] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
    packet[20] = (uint8_t) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
    packet[21] = (uint8_t) ((channels[13] & 0x07FF)>>1);
    packet[22] = (uint8_t) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
    packet[23] = (uint8_t) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
    packet[24] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    
    packet[25] = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3); //CRC

}

void buildElrsPacket(uint8_t packetCmd[],uint8_t command, uint8_t value)
{
  packetCmd[0] = ADDR_MODULE;
  packetCmd[1] = 6; // length of Command (4) + payload + crc
  packetCmd[2] = TYPE_SETTINGS_WRITE;
  packetCmd[3] = ELRS_ADDRESS;
  packetCmd[4] = ADDR_RADIO;
  packetCmd[5] = command;
  packetCmd[6] = value;
  packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);
}

void buildElrsPingPacket(uint8_t packetCmd[])
{
  packetCmd[0] = ADDR_MODULE;
  packetCmd[1] = 4; // length of Command (4) + payload + crc
  packetCmd[2] = TYPE_PING_DEVICES;
  packetCmd[3] = ADDR_BROADCAST;
  packetCmd[4] = ADDR_RADIO;
  packetCmd[5] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);
}
// Request parameter info from known device
//void CRSF_read_param(u8 device, u8 id, u8 chunk) {
void CRSF_read_param(uint8_t packetCmd[],uint8_t id,uint8_t chunk) {
    dbout.printf("read param\n");

    packetCmd[0] = ADDR_MODULE;
    packetCmd[1] = 6; // length of Command (4) + payload + crc
    packetCmd[2] = TYPE_SETTINGS_READ;
    packetCmd[3] = ELRS_ADDRESS;
    packetCmd[4] = ADDR_RADIO;
    packetCmd[5] = id;
    packetCmd[6] = chunk;
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);
}
// request ELRS_info message
void CRSF_get_elrs(uint8_t packetCmd[])
{
  packetCmd[0] = ADDR_MODULE;
  packetCmd[1] = 6; // length of Command (4) + payload + crc
  packetCmd[2] = TYPE_SETTINGS_WRITE;
  packetCmd[3] = ELRS_ADDRESS;
  packetCmd[4] = ADDR_RADIO;
  packetCmd[5] = 0;
  packetCmd[6] = 0;
  packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);
}

void CRSF_sendId(uint8_t packetCmd[],uint8_t modelId ) {
    packetCmd[0] = ELRS_ADDRESS;
    packetCmd[1] = 8; 
    packetCmd[2] = TYPE_COMMAND_ID;
    packetCmd[3] = ELRS_ADDRESS;
    packetCmd[4] = ADDR_RADIO;
    packetCmd[5] = CRSF_SUBCOMMAND;
    packetCmd[6] = COMMAND_MODEL_SELECT_ID;
    packetCmd[7] = modelId;//modelID TODO
    packetCmd[8] = crsf_crc8_BA(&packetCmd[2], packetCmd[1]-2);
    packetCmd[9] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);
}

void CRSF_ping_devices() {
  buildElrsPingPacket(crsfCmdPacket);
  elrsWrite(crsfCmdPacket,6,0);
 }

 

void elrsWrite (uint8_t crsfPacket[],uint8_t size,int32_t add_delay) {
 
  if (crsfPacket[2] != TYPE_CHANNELS) dbout.printf("elrs write 0x%x\n",crsfPacket[2]);

  duplex_set_TX();
  elrs.write(crsfPacket, size);
  elrs.flush();
  //if (add_delay>0)
    //dbout.printf("del:%u\n",add_delay);

  //set last time packet send
  sync_crsf(add_delay);
}

#define MIN(a, b) ((a) < (b) ? a : b)

uint8_t count_params_loaded() {
    int i;
    for (i=0; i < crsf_devices[0].number_of_params; i++) {
        //dbout.printf("count_params_loaded: %i:%u\n",i,crsf_devices[0].number_of_params);
        if (menuItems[i].id == 0) break;
    }
    return i;
}
void sync_crsf (int32_t add_delay) {
  crsfTime = micros();//set current micros
  int32_t offset = (crsfTime-lastCrsfTime);//get dif between pckt send
  uint32_t updated_interval = get_update_interval();
  //debug timing
  #if defined(DEBUG_SYNC)
    if (updated_interval!=20000)
      dbout.printf("%u ; %u ; %i ; %u\n",lastCrsfTime, crsfTime ,offset,updated_interval);
  #endif
  //if (add_delay>0)
    //dbout.printf("delay:%u:%u\n",add_delay,offset);
  crsfTime += ((updated_interval+add_delay) - offset);//set current micros
  lastCrsfTime = crsfTime; //set time that we send last packet
}

uint32_t get_update_interval() {
    if (correction == 0) return updateInterval;

    uint32_t update = updateInterval + correction;
    update = constrain(update, CRSF_FRAME_PERIOD_MIN, CRSF_FRAME_PERIOD_MAX);
    correction -= update - updateInterval;
    return update;
}


void serialEvent() {
  //set uart as rx
  duplex_set_RX();

  while (elrs.available()) {
    if (CRSFframeActive == false)
    {
      unsigned char const inChar = elrs.read();
      // stage 1 wait for sync byte //
      //dbout.write(inChar);
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
        dbout.println("bad packet len");
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
      //////dbout.println(count);

      if (SerialInPacketPtr >= (SerialInPacketLen + 2)) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
      {
        char CalculatedCRC = crsf_crc8(SerialInBuffer + 2, SerialInPacketPtr - 3);
        if (CalculatedCRC == SerialInBuffer[SerialInPacketPtr-1])
        {
          int32_t value;
          uint8_t id = SerialInBuffer[2];
          CRSF_serial_rcv(SerialInBuffer+2, SerialInBuffer[1]-1);


          if (id == CRSF_FRAMETYPE_BATTERY_SENSOR) {
            dbout.print("battery");
            if (getCrossfireTelemetryValue(3, &value, 2)) {
              //batteryVoltage.voltage = value; //todo
              }
            }
          if (id == CRSF_FRAMETYPE_RADIO_ID) {
            //dbout.print("radio id");
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
              dbout.write(SerialInBuffer[i]); 
            }
          #endif
    
        
        } else {
          dbout.write("UART CRC failure");
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


void CRSF_serial_rcv(uint8_t *buffer, uint8_t num_bytes) {
  /*for (int i=0;i<num_bytes;i++) {
    dbout.printf("0x%x:",buffer[i]);
    }
  dbout.println(""); */

  if ((buffer[0]!=CRSF_FRAMETYPE_RADIO_ID) && (buffer[0]!=CRSF_FRAMETYPE_LINK_STATISTICS)){
      //dbout.printf("CRSF FRAMETYPE: 0x%x : L:%u : ",buffer[0],num_bytes);
  } else {
     if (buffer[0]!=CRSF_FRAMETYPE_RADIO_ID) {
      for (int i=0;i<num_bytes;i++) {
        dbout.printf("0x%x:",buffer[i]);
      }
      dbout.println("");
    } 
  }
  switch (buffer[0]) {
    case CRSF_FRAMETYPE_DEVICE_INFO:
       dbout.printf("DEVICE_INFO\n");

        add_device(buffer);
        dbout.printf("send model id\n");

        CRSF_sendId(crsfSetIdPacket,0);
        elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength,0);
        break;

    case CRSF_FRAMETYPE_ELRS_STATUS:
        parse_elrs_info(buffer);
       // dbout.printf("FRAMETYPE_ELRS_STATUS\n");
    
        break;

    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        dbout.printf("PARAMETER_SETTINGS_ENTRY\n");
        add_param(buffer, num_bytes);
        break;

    default:
        break;
    } 
}

void parse_elrs_info(uint8_t *buffer) {
    local_info.bad_pkts = buffer[3];                      // bad packet rate (should be 0)
    local_info.good_pkts = (buffer[4] << 8) + buffer[5];  // good packet rate (configured rate)
  
    // flags bit 0 indicates receiver connected
    // other bits indicate errors - error text in flags_info
    local_info.flags = buffer[6];
    strlcpy(local_info.flag_info, (const char*)&buffer[7], CRSF_MAX_NAME_LEN);  // null-terminated text of flags
  
    local_info.update = elrs_info.update;
    if (memcmp((void*)&elrs_info, (void*)&local_info, sizeof(elrs_info_t)-CRSF_MAX_NAME_LEN)) {
        if (local_info.flag_info[0] && strncmp(local_info.flag_info, elrs_info.flag_info, CRSF_MAX_NAME_LEN)) {
            dbout.printf("error: %s\n",local_info.flag_info);
            //example: error: Model Mismatch
            //error: [ ! Armed ! ]
        }

        memcpy((void*)&elrs_info, (void*)&local_info, sizeof(elrs_info_t)-CRSF_MAX_NAME_LEN);
        elrs_info.update++;
   }
   
    //example bad_pckts : good_pckts ; flag ; flag_info ; info_update ;
    // 0 : 100 ; 5 ; Model Mismatch ; 0 
    // 0 : 200 ; 8 ; [ ! Armed ! ] ; 0 
    dbout.printf("%u : %u ; %u ; %s ; %u\n ",local_info.bad_pkts,local_info.good_pkts,local_info.flags,local_info.flag_info,local_info.update);
}



uint8_t getCrossfireTelemetryValue(uint8_t index, int32_t *value, uint8_t len) {
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
void add_device(uint8_t *buffer) {
            
    for (int i=0; i < CRSF_MAX_DEVICES; i++) {
        if (crsf_devices[i].address == buffer[2]        //  device already in table
         || crsf_devices[i].address == 0                //  not found, add to table
         || crsf_devices[i].address == ADDR_RADIO) {    //  replace deviation device if necessary
            dbout.printf("no devices match : parse new:0x%x\n",buffer[2]);

            parse_device(buffer, &crsf_devices[i]);
            break;
        }
    }
    //  no new device added if no more space in table
}



void add_param(uint8_t *buffer, uint8_t num_bytes) {
  // abort if wrong device, or not enough buffer space
  dbout.printf("add_param:%u:%u:0x%x\n",buffer[3],crsf_devices[0].number_of_params,crsf_devices[device_idx].address);
  //CRSF_ADDRESS_CRSF_TRANSMITTER 
  if (buffer[2] != crsf_devices[device_idx].address
       || ((int)((sizeof recv_param_buffer) - (recv_param_ptr - recv_param_buffer)) < (num_bytes-4))) {
        recv_param_ptr = recv_param_buffer;
        next_chunk = 0;
        next_param = 0;
        dbout.println("err");
        dbout.println(recv_param_ptr);
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
            dbout.printf("next chunk :: %u:%u",next_param, next_chunk);

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
    menuItems[buffer[3]-1].displayInfo();


    recv_param_ptr = recv_param_buffer;
    next_chunk = 0;
    params_loaded = count_params_loaded();
    // read all params when needed
    dbout.printf("params_loaded: %u\n",params_loaded);
    if (params_loaded < crsf_devices[device_idx].number_of_params)
       // && (menu_item_id+submenu_item_id <  crsf_devices[device_idx].number_of_params))
    {
      if (next_param < crsf_devices[device_idx].number_of_params)
      {
        next_param += 1;
      } else {
        next_param = 1;
      }
       // dbout.printf("count_out:%u:%u:%u\n",
     // device_idx,
      //  crsf_devices[device_idx].number_of_params,params_loaded);
      //dbout.printf("id:%u:%s\n",parameter->id,parameter->name);

        CRSF_read_param(crsfCmdPacket, next_param, next_chunk);
        elrsWrite(crsfCmdPacket,8,0); 
  } else {
    /*  dbout.printf("0_count_out:%u:%u:%u\n",
        device_idx,
        crsf_devices[device_idx].number_of_params,
        params_loaded); */
       
    next_param = 0; 
    }
}


uint32_t parse_u32(const uint8_t *buffer) {
    return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
}
void parse_device(uint8_t* buffer, crsf_device_t *device) {
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
        dbout.println("Module type: elrs");
        protocol_module_type(MODULE_ELRS);
      }
        else
      {
        dbout.println("Module type: not elrs");
        protocol_module_type(MODULE_OTHER);
      } 
    }
    dbout.printf("Module details:%s,0x%x,%u,%u,%u,%u,%u\n",
                    device->name,
                    device->address,
                    device->number_of_params,
                    device->params_version,
                    device->serial_number,
                    device->firmware_id,
                    device->hardware_id);
  dbout.printf("devices: %u : %u : 0x%x\n",device_idx, crsf_devices[device_idx].number_of_params,crsf_devices[device_idx].address);
  
}


