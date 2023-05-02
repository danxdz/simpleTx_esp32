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

#pragma once

#include <Arduino.h>
#include "crsf_protocol.h"

typedef struct {
  int aileron = 0;
  int elevator = 0; 
  int throttle = 0;
  int rudder = 0;
  int aux1 = 0;
  int aux2 = 0;
  int aux3 = 0;
  int aux4 = 0;
} rc_input_t;

#define CRSF_MAX_PARAMS 100 // one extra required, max observed is 47 in Diversity Nano RX
#define CRSF_MAX_DEVICES 4

#define CRSF_MAX_STRING_BYTES 2500 // max observed is 2010 in Nano RX
#define CRSF_STRING_BYTES_AVAIL(current) (CRSF_MAX_STRING_BYTES - ((char *)(current)-mp->strings))

typedef struct
{
    uint8_t address;
    uint8_t number_of_params;
    uint8_t params_version;
    uint32_t serial_number;
    uint32_t hardware_id;
    uint32_t firmware_id;
    char name[CRSF_MAX_NAME_LEN];
} crsf_device_t;

typedef enum
{
    MODULE_UNKNOWN,
    MODULE_ELRS,
    MODULE_OTHER,
} module_type_t;

uint8_t protocol_module_is_elrs();

#define CRSF_MAX_CHANNEL 16

extern int rcChannels[CRSF_MAX_CHANNEL];

#define CRSF_MAX_CHUNK_SIZE 58 // 64 - header - type - destination - origin
#define CRSF_MAX_CHUNKS 8     // not in specification. Max observed is 3 for Nano RX

extern module_type_t module_type;
extern uint8_t device_idx; // current device index

extern char recv_param_buffer[];
extern char *recv_param_ptr;

// Basic setup
#ifdef DEBUG
#define SERIAL_BAUDRATE 115200 // low baud for Arduino Nano , the TX module will auto detect baud. max packet rate is 250Hz.
#else
#define SERIAL_BAUDRATE 400000 // testing 3750000//1870000
#endif
// Device address & type
#define RADIO_ADDRESS 0xEA
#define ADDR_MODULE 0xEE //  Crossfire transmitter
#define TYPE_CHANNELS 0x16

// Define RC input limite
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_FRAME_PERIOD_MIN 850   // 1000Hz 1ms, but allow shorter for offset cancellation
#define CRSF_FRAME_PERIOD_MAX 50000 // 25Hz  40ms, but allow longer for offset cancellation
// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US 1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US 5000   // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE 26
#define CRSF_FRAME_LENGTH 24 // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE 8
#define LinkStatisticsFrameLength 10 //

// ELRS command
#define ELRS_ADDRESS 0xEE
#define ELRS_RX_ADDRESS 0xEC
#define ELRS_BIND_COMMAND 0xFF
#define ELRS_WIFI_COMMAND 0xFE
#define ELRS_PKT_RATE_COMMAND 1
#define ELRS_TLM_RATIO_COMMAND 2
#define ELRS_POWER_COMMAND 3

#define ADDR_RADIO 0xEA //  Radio Transmitter

// Frame Type
#define TYPE_GPS 0x02
#define TYPE_VARIO 0x07
#define TYPE_BATTERY 0x08
#define TYPE_HEARTBEAT 0x0b
#define TYPE_VTX 0x0F
#define TYPE_VTX_TELEM 0x10
#define TYPE_LINK 0x14
#define TYPE_CHANNELS 0x16
#define TYPE_RX_ID 0x1C
#define TYPE_TX_ID 0x1D
#define TYPE_ATTITUDE 0x1E
#define TYPE_FLIGHT_MODE 0x21
#define TYPE_PING_DEVICES 0x28
#define TYPE_DEVICE_INFO 0x29
#define TYPE_REQUEST_SETTINGS 0x2A
#define TYPE_SETTINGS_ENTRY 0x2B
#define TYPE_SETTINGS_READ 0x2C
#define TYPE_SETTINGS_WRITE 0x2D
#define TYPE_ELRS_INFO 0x2E
#define TYPE_COMMAND_ID 0x32
#define TYPE_RADIO_ID 0x3A

// Frame Subtype
#define UART_SYNC 0xC8
#define CRSF_SUBCOMMAND 0x10
#define COMMAND_MODEL_SELECT_ID 0x05

#define TELEMETRY_RX_PACKET_SIZE 64
#define CRSF_MAX_FIXEDID 63
#define CRSF_CRC_POLY 0xd5

#define CRSF_MAX_PACKET_LEN 64

#define SEND_MSG_BUF_SIZE 64 // don't send more than one chunk
#define ADDR_BROADCAST 0x00  //  Broadcast address

#define MODULE_IS_ELRS (module_type == MODULE_ELRS)
#define MODULE_IS_UNKNOWN (module_type == MODULE_UNKNOWN)

typedef struct
{
    uint8_t update;
    uint8_t bad_pkts;
    uint16_t good_pkts;
    uint8_t flags;
    char flag_info[CRSF_MAX_NAME_LEN];
} elrs_info_t;

uint32_t parse_u32(const uint8_t *buffer);
void parse_device(uint8_t *buffer, crsf_device_t *device);

/// UART Handling ///
static volatile uint8_t SerialInPacketLen; // length of the CRSF packet as measured
static volatile uint8_t SerialInPacketPtr; // index where we are reading/writing
static volatile bool CRSFframeActive;      // = false; //since we get a copy of the serial data use this flag to know when to ignore it

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);
void crsfSendChannels(rc_input_t* rc_input);

void buildElrsPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);
void CRSF_broadcast_ping();

void CRSF_read_param(uint8_t n_param, uint8_t n_chunk, uint8_t target);

// void CRSF_get_elrs_info(uint8_t packetCmd[]);
void CRSF_get_elrs_info(uint8_t target);

void CRSF_send_id(uint8_t modelId);

void CRSF_write(uint8_t crsfPacket[], uint8_t size, int32_t add_delay);

uint8_t count_params_loaded(uint8_t index);

void protocol_module_type(module_type_t type);

void sync_crsf(int32_t add);
uint32_t get_update_interval();
void serialEvent();
void CRSF_serial_rcv(uint8_t *buffer, uint8_t num_bytes);
uint8_t getCrossfireTelemetryValue(uint8_t index, int32_t *value, uint8_t len);

void add_device(uint8_t *buffer);
void parse_elrs_info(uint8_t *buffer);
void add_param(uint8_t *buffer, uint8_t num_bytes);
void CRSF_changeParam(uint8_t n_param, uint8_t n_chunk);

extern uint8_t rxConected;
extern uint8_t txConected;
extern elrs_info_t local_info;
extern elrs_info_t elrs_info;

extern uint32_t crsfTime;
extern uint32_t lastCrsfTime;
extern uint32_t updateInterval;
extern int32_t correction;

extern crsf_device_t crsf_devices[];

extern volatile crsf_sensor_battery_s batteryVoltage;
extern crsfPayloadLinkstatistics_s LinkStatistics; // Link Statisitics Stored as Struct
