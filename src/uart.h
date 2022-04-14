#include "Arduino.h"
#include "CrsfProtocol/crsf_protocol.h"
/// UART Handling ///
volatile uint8_t SerialInPacketLen = 0; // length of the CRSF packet as measured
volatile uint8_t SerialInPacketPtr = 0; // index where we are reading/writing
volatile bool CRSFframeActive = false; //since we get a copy of the serial data use this flag to know when to ignore it

crsfPayloadLinkstatistics_s LinkStatistics; // Link Statisitics Stored as Struct
static volatile crsf_sensor_battery_s batteryVoltage;

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
