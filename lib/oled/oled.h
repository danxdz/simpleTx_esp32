#ifndef OLED_H /* include guards */
#define OLED_H


#include "crsf_protocol.h"
//#include <U8x8lib.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

//U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);


class Oled {
  
  public:
      
    void init();
    void setMsg(char *msg);
    void setMainScreen(char *name, crsfLinkStatistics_t LinkStats,uint8_t bpkts, uint8_t gpkts);
    void setMainMenuItems();
    void setSubMenuItems();
    void printf(char* tmp);
    void PrintCenter(uint8_t y,char * tmp);
    void PrintRight(char *tmp);
    void PrintRight(uint8_t y,char *tmp);

};

#endif