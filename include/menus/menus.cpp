
//menu code based on work from:
//https://www.youtube.com/watch?v=fjsmVzMdqyU
//https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
//dsnmechanic@gmail.com 
//instagram: dsnmechanics

#include <Arduino.h>
#include "menus.h"

#include "oled/oled.cpp"
#include "gpio/gpio.cpp"

void Menu::ChangeParam(uint8_t param, uint8_t cmd){
  db_out.printf("ChangeParam: %s\n",menuItems[selected].name);
  buildElrsPacket(crsfCmdPacket,param,cmd);
  elrsWrite(crsfCmdPacket,8,200000);

  delay(500);

  next_chunk = 0;
  CRSF_read_param(crsfCmdPacket,param,next_chunk);
  elrsWrite(crsfCmdPacket,8,20000); 
}


void Menu::loadMainMenu(char *load) {
  display.clearDisplay();
  display.drawStr(0,8,load);
  display.sendBuffer();
  delay(500);
}