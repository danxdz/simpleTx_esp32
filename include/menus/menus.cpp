
//menu code based on work from:
//https://www.youtube.com/watch?v=fjsmVzMdqyU
//https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
//dsnmechanic@gmail.com 
//instagram: dsnmechanics

#include <Arduino.h>
#include "oled.h"
#include "gpio/gpio.cpp"

 

void displayMenu(char * name,menu_items *mItem,int num) { 
  display.println(name);

  
  char * menu_item;

  int page = (selected/5);
  int offset = num-5;
  int start = page*5;
  int max = (num-offset)*(page+1);
  if (max>num) max = num;
  if (selected>=num) selected = 0;

  //db_out.printf("zerzer");

  display.printf("%i:%i:%i:%i:%i \n",
  selected,offset,max,page,start);
  
  for (int i = start; i < max; i++) {
      if (i == selected)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      else if (i != selected)
        display.setTextColor(SSD1306_WHITE);
    
      menu_item = mItem[i].name;
      //TODO options
      //db_out.printf("%s:%i:%u\n",menu_item,i,mItem[i].id);
      display.printf("%s\n",menu_item,i,mItem[i].id);
    
  }
  if (offset>0) {
    display.setTextColor(SSD1306_WHITE);
    display.printf("... \n");
  } 
}
void displaySubmenu(menu_items *mItems) { 
 //db_out.printf("display submenu\n");
  
  display.println("CRSF config");
  display.printf("%s:%u\n",mItems[selected].name,mItems[selected].opt_count);
  //db_out.printf("%s: %u\n",mItems[selected].name,mItems[selected].opt_count);

  for (int i = 0; i < 20; i++)
  {
    //db_out.printf("%u\n",mItems[selected].opt_list[i]);
  
    if (mItems[selected].opt_list[i]) {  // && mItems[selected].opt_count ) {
     // db_out.printf("%i:%s:%u:%i:%i\n",i,
     // mItems[selected].opt_list[i],mItems[selected].u.status,selected,subSelected);
      if (subSelected==-1) subSelected = mItems[selected].u.status;
      

      if (i == subSelected)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      else if (i != subSelected)
        display.setTextColor(SSD1306_WHITE);
    
    
      display.printf("%s\n",mItems[selected].opt_list[i]);
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
                    int num_menu_item,
                    menu_items *mItems,
                    int entered  ) {

display.clearDisplay();
display.setTextSize(1);
display.setTextColor(SSD1306_WHITE);
display.setCursor(0, 0);
     
  if (entered == -1) {
    //db_out.printf("name:%s,mItems:%u,num_menu_item%i\n",name,mItems[0].name,num_menu_item);
    displayMenu(name,mItems,num_menu_item);
  }
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
    displaySubmenu(mItems);
  }
  display.display();
  delay(200);
}