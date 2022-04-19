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

 
 // Basic setup
#define CRSF_MAX_CHANNEL 16
#ifdef DEBUG
    #define SERIAL_BAUDRATE 115200 //low baud for Arduino Nano , the TX module will auto detect baud. max packet rate is 250Hz.
#else
    #define SERIAL_BAUDRATE 3750000//1870000 //testing
#endif
 // Device address & type
#define RADIO_ADDRESS                  0xEA
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS                  0x16

// Define RC input limite
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811

//Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_FRAME_PERIOD_MIN     850    // 1000Hz 1ms, but allow shorter for offset cancellation
#define CRSF_FRAME_PERIOD_MAX     50000  // 25Hz  40ms, but allow longer for offset cancellation
// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     5000 // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24;   // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE  8
#define LinkStatisticsFrameLength 10 //

// ELRS command
#define ELRS_ADDRESS                   0xEE
#define ELRS_BIND_COMMAND              0xFF
#define ELRS_WIFI_COMMAND              0xFE
#define ELRS_PKT_RATE_COMMAND          1
#define ELRS_TLM_RATIO_COMMAND         2
#define ELRS_POWER_COMMAND             3
#define TYPE_SETTINGS_WRITE            0x2D
#define TYPE_SETTINGS_READ             0x2C
#define ADDR_RADIO                     0xEA  //  Radio Transmitter

// Frame Type
#define TYPE_GPS              0x02
#define TYPE_VARIO            0x07
#define TYPE_BATTERY          0x08
#define TYPE_HEARTBEAT        0x0b
#define TYPE_VTX              0x0F
#define TYPE_VTX_TELEM        0x10
#define TYPE_LINK             0x14
#define TYPE_CHANNELS         0x16
#define TYPE_RX_ID            0x1C
#define TYPE_TX_ID            0x1D
#define TYPE_ATTITUDE         0x1E
#define TYPE_FLIGHT_MODE      0x21
#define TYPE_PING_DEVICES     0x28
#define TYPE_DEVICE_INFO      0x29
#define TYPE_REQUEST_SETTINGS 0x2A
#define TYPE_SETTINGS_ENTRY   0x2B
#define TYPE_SETTINGS_READ    0x2C
#define TYPE_SETTINGS_WRITE   0x2D
#define TYPE_ELRS_INFO        0x2E
#define TYPE_COMMAND_ID       0x32
#define TYPE_RADIO_ID         0x3A

// Frame Subtype
#define UART_SYNC                      0xC8
#define CRSF_SUBCOMMAND                0x10
#define COMMAND_MODEL_SELECT_ID        0x05

#define TELEMETRY_RX_PACKET_SIZE   64



#define CRSF_CRC_POLY 0xd5

#define CRSF_MAX_PACKET_LEN            64



#define CRSF_MAX_CHUNK_SIZE   58   // 64 - header - type - destination - origin
#define CRSF_MAX_CHUNKS        5   // not in specification. Max observed is 3 for Nano RX

#define SEND_MSG_BUF_SIZE  64      // don't send more than one chunk
#define ADDR_BROADCAST  0x00  //  Broadcast address
