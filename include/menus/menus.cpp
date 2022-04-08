
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
  db_out.printf("display Menu\n");

  db_out.printf("name:%s,num_menu_item: %i\n",name,num);
  
  char * menu_item;
  int menu_item_num = 20;

  //menu_items valid_items[55]; 
  //memset(valid_items,0,sizeof valid_items);

  for (int i=0; i<num ; i++) {
    
  //  menu_items *vItem = &valid_items[menu_item_num];
    if (mItem[i].parent == 0 ) {
      //memcpy ( &vItem, &mItems[i], sizeof(mItems[i]) );
      //menu_item_num++;
      db_out.printf("item:%i:%s:%u\n",
    mItem[i].id,mItem[i].name,mItem[i].parent);
    }
  }  
  //db_out.printf("end:\n");

  int start = (selected/5)*5;
  int max = (menu_item_num-(menu_item_num-5))*((selected/5)+1);
  if (max>menu_item_num) max = menu_item_num;
  if (selected>=menu_item_num) selected = 0;


  display.printf("%i:%i:%i:%i:%i \n",
  selected,max,start,menu_item_num,num);
  
  for (int i = start; i < max; i++) {
    if (mItem[i].parent == 0 ) {
      if (i == selected) 
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        else if (i != selected)
        display.setTextColor(SSD1306_WHITE);
      
        menu_item = mItem[i].name;
        //TODO options
        db_out.printf("%s:%i:%u:%s:%u\n",menu_item,i,mItem[i].id,mItem[i].name,mItem[i].parent);
        display.printf("%s\n",menu_item);
    } else {
      i++;
    }
  }
  if ((menu_item_num-5)>0) {
      display.setTextColor(SSD1306_WHITE);
      display.printf("... \n");
  }  
}
void displaySubmenu(menu_items *mItem) { 
 //db_out.printf("display submenu\n");
  
  display.println("CRSF config");
  display.printf("%s:%u\n",
  mItem[selected].name,
  mItem[selected].max_value);
  /* db_out.printf("%s: %u\n",
  mItem[selected].name,
  mItem[selected].max_value);
 */
  int max = mItem[selected].max_value;
  if (max == 0) {
    max++;
  }

  for (int i = 0; i <= max; i++)
  {
    //db_out.printf("%u\n",mItems[selected].opt_list[i]);
  
  //  if (mItem[selected].submenu_item[i]) {  // && mItems[selected].opt_count ) {
     /*  db_out.printf("%i:%s:st:%u:par:%u:%i:%i:%i\n",
      i,
      mItem[selected].opt_list[i],
      mItem[selected].u.status,
      mItem[selected].parent,
      mItem[selected].max_value,
      selected,
      subSelected);   */

      if (subSelected==-1) subSelected = mItem[selected].u.status;
      

      if (i == subSelected)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      else if (i != subSelected)
        display.setTextColor(SSD1306_WHITE);
    
    
      display.printf("%s\n",mItem[selected].submenuItems[i]);
   // }
    
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
                    menu_items *mItem,
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
    displaySubmenu(mItem);
  }
  display.display();
  delay(200);
}