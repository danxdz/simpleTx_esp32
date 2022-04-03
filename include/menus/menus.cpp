
//menu code based on work from:
//https://www.youtube.com/watch?v=fjsmVzMdqyU
//https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
//dsnmechanic@gmail.com 
//instagram: dsnmechanics

#include <Arduino.h>
#include "oled.h"
#include "gpio/gpio.cpp"

 

void displayMenu(char * name,crsf_param_t *crsf_p) { 
  display.println(name);
  display.println("");
  char *menu_item;
  for (int i = 0; i < 20; i++) {
    if (crsf_p[i].parent == 0) {
      menu_item = crsf_p[i].name;
      if (i == selected)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      else if (i != selected)
        display.setTextColor(SSD1306_WHITE);
      display.printf("%s   <1>\n",menu_item);
    }
  }
}
void displaySubmenu() { 
    display.println("test");
    display.printf("Menu option %i\n",selected+1);
    display.println("");
    if (updated==0){
      updated=1;
      for (int i=0;i<=3;i++) {
        //Serial.printf("%i:%i\n",selected,i);
        display.println("submenu");
      }
    }
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
                    uint16_t gpkts,
                    char * name,
                    module_type_t typeModule,
                    int params_loaded,
                    crsf_param_t *crsf_params,
                    int entered  ) {

display.clearDisplay();
display.setTextSize(1);
display.setTextColor(SSD1306_WHITE);
display.setCursor(0, 0);
   
  if (entered == -1) 
    displayMenu(name,crsf_params);
  else if (entered == -2) {
    //display.println(F("main"));
    //updateDisplay();
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    char * typeModName;
      if (typeModule==1) typeModName =  (char *)"elrs";
    display.printf("%s:%s",name,typeModName);
    display.println("");
    display.printf("Tx %idBm %i:%i%% ",tx_rssi,rf_mode,tx_lq);
    display.println("");
    display.println("");
    display.printf("Rx %idBm %i:%i%% ",rx_rssi_1,rf_mode,rx_lq);
    display.setTextSize(2);
    display.println("");
    display.printf("%imW", tx_pwr);
    float vBat = (float)batteryVoltage/10;
    display.printf("%4.1fv",vBat);
    display.println();             // Start at top-left corner
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.printf("%u:%u",bpkts,gpkts);
  } else {
    displaySubmenu();
  }
  display.display();
  //delay(200);
}