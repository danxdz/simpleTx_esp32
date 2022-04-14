
//menu code based on work from:
//https://www.youtube.com/watch?v=fjsmVzMdqyU
//https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
//dsnmechanic@gmail.com 
//instagram: dsnmechanics

#include <Arduino.h>
#include "menus.h"

#include "oled.h"
#include "gpio/gpio.cpp"

 
int num_lines = 5;

void displayMenu(char * name,menu_items *mItem,int menu_item_num) { 
  display.println(name);
 
  char * menu_item;
  char * option_items;
   
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
              char *start = mItem[i].name;
              for (char *p = (char *)mItem[i].name; *p; p++) {
                if (*p == ' ') {
                    int len = (strlen(start)-strlen(p));
                    menu_item = new char[len+3];
                    strncpy(menu_item,start,len+3);
                    menu_item[len+2] = '.' ;
                    menu_item[len+3] = '\0' ;
                    //db_out.printf("llenn: %i:%s:%i\n", len,menu_item,strlen(menu_item));

                }
                   // strlcpy(mItemP->submenuItems[count],start,len+1);
              }
              display.printf("%s",menu_item);

             } else display.printf("%s",mItem[i].name);

          int tmp = 128-(strlen(mItem[i].submenuItems[mItem[i].u.status])*6);
            //db_out.printf("%s:%i\n",mItem[i].submenuItems[mItem[i].u.status],tmp);
          display.setCursor(tmp,display.getCursorY());
          display.printf("%s\n",mItem[i].submenuItems[mItem[i].u.status]);

        } else {
          if (strlen(mItem[i].name) > 20) {
            char *start = mItem[i].name;
            for (char *p = (char *)mItem[i].name; *p; p++) {
              if (*p == ' ') {
                int len = (strlen(start)-strlen(p));
                menu_item = new char[len+3];
                strncpy(menu_item,start,len+3);
                menu_item[len+2] = '.' ;
                menu_item[len+3] = '\0' ;
                //db_out.printf("llenn: %i:%s:%i:%s:%s\n", len,menu_item,strlen(menu_item),start,p);
                // strlcpy(mItemP->submenuItems[count],start,len+1);
                start = p+1;
                break;
              }
            }
            display.printf("%s",menu_item);

            for (char *p = (char *)start; *p; p++) {
              if (*p == ' ') {
                start = p+1;
              }
            }     
            int len = strlen(start);
            option_items = new char[len+1];
            strncpy(option_items,start,len+1);
         
            int tmp = (128-(strlen(option_items)+strlen(menu_item))*4)-5;
           /*  db_out.printf("ln: %s:%i:%i:%i\n",
            option_items,strlen(option_items),strlen(menu_item),tmp);
             */
            display.setCursor(tmp,display.getCursorY());
         
            display.printf("%s\n",option_items);

          } else display.printf("%s\n",mItem[i].name);
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
  int start_item_index = 20;
  for (int i = 0 ; smItem[i].parent != 0 ; i++){
    if (smItem[i].parent == mItem[selected].id) {
      if (start_item_index>i ) 
        start_item_index = i;
     
      if (i == subSelected)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      else if (i != subSelected)
        display.setTextColor(SSD1306_WHITE);
    
      display.printf("%s",smItem[i].name);
      if (smItem[i].submenuItems[0]) {
        display.setCursor(80,display.getCursorY());
        display.printf("%s\n",smItem[i].submenuItems[smItem[i].u.status]);
      } else {
        display.println("");
      }
      count_item++;
    }
  }













  
 /*  db_out.printf("%i:%i:%i:%i\n",
      selected,
      subSelected,
      count_item,
      start_item_index);  */
  if ((subSelected-start_item_index)+1 > count_item) subSelected = start_item_index;
  if ((subSelected) < start_item_index ) subSelected = (start_item_index+count_item)-1;
}

void updateDisplay(
                    crsfPayloadLinkstatistics_s LinkStatistics, 
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
    display.printf("Tx %idBm %i:%i%% ",
                      LinkStatistics.uplink_RSSI_1,
                      LinkStatistics.rf_Mode,
                      LinkStatistics.uplink_Link_quality);
    display.println("");
    display.println("");
    display.printf("Rx %idBm %i:%i%% ",
                      LinkStatistics.downlink_RSSI,
                      LinkStatistics.rf_Mode,
                      LinkStatistics.downlink_Link_quality);
    display.setTextSize(2);
    display.println("");
    display.printf("%imW", LinkStatistics.uplink_TX_Power);
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


void set_display_loading(char *load) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("%s",load);
  display.display();
  delay(500);
}

void check_too_big (char *tmp) {

  
}