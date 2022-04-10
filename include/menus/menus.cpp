
//menu code based on work from:
//https://www.youtube.com/watch?v=fjsmVzMdqyU
//https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
//dsnmechanic@gmail.com 
//instagram: dsnmechanics

#include <Arduino.h>
#include "oled.h"
#include "gpio/gpio.cpp"

 
int num_lines = 5;
void displayMenu(char * name,menu_items *mItem,int menu_item_num) { 
  display.println(name);
 
  char * menu_item;
   
  int start = (selected/num_lines)*num_lines;
  int max = (menu_item_num-(menu_item_num-num_lines))*((selected/num_lines)+1);
  if (max>menu_item_num) max = menu_item_num;
  if (selected>=menu_item_num) selected = 0;


  display.printf("%i:%i:%i:%i\n",
  selected,max,start,menu_item_num);
  
  for (int i = start; i < max; i++) {
      if (i == selected) 
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        else if (i != selected)
        display.setTextColor(SSD1306_WHITE);
        
        //TODO options
       // db_out.printf("%s:%i:%u:%s:%u\n",menu_item,i,mItem[i].id,mItem[i].name,mItem[i].parent);
            //menu_item = strndup(menu_item,6);
            //menu_item = (char *)"12345678";
        
        if ((mItem[i].submenuItems[0])&&(mItem[i].submenuType==0)) {
          if ((strlen(mItem[i].name)+strlen(mItem[i].submenuItems[mItem[i].u.status])) > 20) 
             {
             //  int len = 20-strlen(mItem[i].submenuItems[mItem[i].u.status]); 
              //db_out.printf("llenn: %i\n", len);
             // menu_item = new char(len);
            //  strncpy(menu_item , mItem[i].name, len-1);
            //  menu_item[len-1] = '\0' ;
            
              //display.printf("%s",menu_item);
              display.printf("%s\n",mItem[i].name);

             } else display.printf("%s",mItem[i].name);

          int tmp = 128-(strlen(mItem[i].submenuItems[mItem[i].u.status])*6);
            //db_out.printf("%s:%i\n",mItem[i].submenuItems[mItem[i].u.status],tmp);
          display.setCursor(tmp,display.getCursorY());
          display.printf("%s\n",mItem[i].submenuItems[mItem[i].u.status]);

        } else {
          display.printf("%s\n",mItem[i].name);
        }
  } 
  
  if ((menu_item_num-num_lines)>0) {
      display.setTextColor(SSD1306_WHITE);
      display.printf("... \n");
  }  
}
void displaySubmenu(menu_items *mItem,menu_items *smItem) { 
 //db_out.printf("display submenu\n");
  
  display.println("CRSF config");
  display.printf("%s:%u\n",
  mItem[selected].name,
  mItem[selected].max_value);
  /* db_out.printf("%s: %u\n",
  mItem[selected].name,
  mItem[selected].max_value);
 */
    int count_item = 0;
    int min_item = 20;
    for (int i = 0 ; smItem[i].parent != 0 ; i++){
      if (smItem[i].parent == mItem[selected].id) {
        if (min_item>i ) min_item = i;
        if (subSelected==-1) {
          subSelected = i;
          //min_item = i;
          }
        if (i == subSelected)
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        else if (i != subSelected)
          display.setTextColor(SSD1306_WHITE);
      
      
      /*   db_out.printf("%i:%i:%i:%s:%i:%i:%i\n",
        i,
        selected,
        subSelected,
        smItem[i].name,
        smItem[i].max_value,
        count_item,
        min_item); */

      display.printf("%s",smItem[i].name);
      display.setCursor(80,display.getCursorY());
     
      display.printf("%u\n",smItem[i].u.text_sel);
      count_item++;
      }

    }
    if (subSelected>count_item+1 ) subSelected = min_item;
    if (subSelected<min_item ) subSelected = count_item+1;
 
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
                    menu_items *mItem,
                    menu_items *smItem,
                    int entered  ) {

display.clearDisplay();
display.setTextSize(1);
display.setTextColor(SSD1306_WHITE);
display.setCursor(0, 0);
     
  if (entered == -1) {
    displayMenu(name,mItem,num_menu_item);
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
    displaySubmenu(mItem,smItem);
  }
  display.display();
  delay(50);
}