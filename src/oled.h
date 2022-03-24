#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include "CrsfProtocol/crsf_protocol.h"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16



void startDisplay() {
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  
  display.setCursor(0,0);             // Start at top-left corner
  display.setTextColor(SSD1306_WHITE); 
  display.printf("starting...");
  display.display();
}
void updateDisplay(
                    int8_t tx_rssi,
                    uint8_t tx_lq,
                    uint8_t rf_mode,
                    uint8_t tx_pwr,
                    int8_t rx_rssi_1,
                    int8_t rx_rssi_2,
                    uint8_t rx_lq, 
                    uint8_t batteryVoltage,
                    uint8_t bpkts,
                    uint16_t gpkts ) {
  
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  //int32_t tx_rssi_1 = tx_rssi;//LinkStatistics.uplink_RSSI_1;
  //int8_t rf_mode = 5;//LinkStatistics.rf_Mode;
  //int8_t tx_lq = tx_lq; //LinkStatistics.uplink_Link_quality;
  display.printf("Tx %idBm %i:%i%% ",tx_rssi,rf_mode,tx_lq);
  display.println("");
  //int32_t rx_rssi = 100;//LinkStatistics.downlink_RSSI;
  //int8_t rx_lq = 100;//LinkStatistics.downlink_Link_quality;
  display.printf("Rx %idBm %i:%i%% ",rx_rssi_1,rf_mode,rx_lq);
  display.setTextSize(2);             // Normal 1:1 pixel scale
  //int8_t tx_power = 10;// LinkStatistics.uplink_TX_Power;
  display.println("");
  display.printf("%imW", tx_pwr);
  float vBat = (float)batteryVoltage/10;
  display.printf("%4.1fv",vBat);

  display.println();             // Start at top-left corner
  display.setTextSize(1);             // Normal 1:1 pixel scale

  display.printf("%u:%u",bpkts,gpkts);

  display.display();
}
