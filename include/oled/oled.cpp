#include "Arduino.h"
#include "oled.h"
#include "xbmStrings.h"

#define DEBUG


void Oled::PrintCenter(uint8_t y,char *tmp) {
    uint8_t x = (display.getDisplayWidth()/2) - (display.getStrWidth(tmp)/2);
    display.drawStr(x,y,tmp);
}
void Oled::PrintRight(char *tmp) {
    uint8_t x = display.getDisplayWidth() - display.getStrWidth(tmp);
    display.drawStr(x,display.getCursorY(),tmp);
}

void Oled::PrintRight(uint8_t y,char *tmp) {
    uint8_t x = display.getDisplayWidth() - (display.getStrWidth(tmp));
    display.drawStr(x,y,tmp);
}



void Oled::init() {
    #if defined(DEBUG) 
        db_out.printf("starting screen..."); 
    #endif

    display.begin();
    display.clearBuffer(); 
    display.setFont(u8g2_font_minicute_tr);
    Oled::PrintCenter(7,(char *)"simpleTX");

    display.drawXBM(48, 15, 32, 32, elrs32);  
    
    display.setFont(u8g2_font_5x7_mr);
    
    Oled::PrintCenter(58,(char *)"ExpressLRS");
    display.sendBuffer();
    delay(4000);
}


void Oled::setMainScreen(char *name, crsfLinkStatistics_t LinkStatistics,uint8_t bpkts,uint8_t gpkts) {
    //db_out.printf("Mainsreen\n");
    display.clearBuffer(); 
    display.setFont(u8g2_font_chikita_tr);
    display.drawStr(0,7, name);
    display.setCursor(0,14);
    //display.print("tx ");
    char output[400];
    snprintf(output, sizeof output, "%u:%u | %idBm  \n", 
        LinkStatistics.rf_Mode,
        LinkStatistics.uplink_Link_quality,
        LinkStatistics.uplink_RSSI_1);
    display.print(output);
    display.setCursor(0,55);    
    //display.print("rx ");
    snprintf(output, sizeof output, "%u:%u | %idBm  \n", 
        LinkStatistics.rf_Mode,
        LinkStatistics.downlink_Link_quality,
        LinkStatistics.downlink_RSSI);
    display.print(output);
   
    display.setCursor(0,64); 
    snprintf(output, sizeof output, "%u:%u", 
        bpkts,
        gpkts);   
    display.print(output);

    display.setFont(u8g2_font_9x15_me);
    display.setCursor(90,10);    
    display.print(LinkStatistics.uplink_TX_Power);
    display.print("mW");

    display.setFont(u8g2_font_10x20_mr);
    display.setCursor(0,35);    
    float vBat = (float)batteryVoltage.voltage/10;
    display.print(vBat,2);  
    display.setFont(u8g2_font_5x7_mr);
     
    display.sendBuffer();


}

void Oled::setSubMenuItems() {
    db_out.printf("Subscreen: %u\n ", display.getWidth());
    display.clearBuffer(); 
    display.setFont(u8g2_font_profont11_tr);
    display.setCursor(0,7);
    display.print(crsf_devices[0].name);
    display.setCursor(0,15);
    display.print(menuItems[selected].name);

     //debug submenus options
    #if !defined(DEBUG) 
        display.print("CRSF settings");  
    #else 
        display.print(":");
        display.print(selected);  
        display.print(":");
        display.print(subSelected);
    #endif
    for (size_t i = 0; i<crsf_devices[0].number_of_params; i++) {
        if (menuItems[i].parent == menuItems[selected].id ) {
            db_out.printf("%s:%u\n",menuItems[i].name,menuItems[i].id); 
            display.setCursor(0,display.getCursorY()+9);
            
            if (i == subSelected) 
                display.print("> ");
         
            display.print(menuItems[i].name);
            //db_out.printf("len! %u;%u\n",len,  display.getDisplayWidth());
            //display.setFont(u8g2_font_profont10_mr);

            Oled::PrintRight(menuItems[i].optionsMainMenu[menuItems[i].status]);
            
        }
    }
    display.sendBuffer();
}






void Oled::setMainMenuItems() {
    /* db_out.printf("Mainsreen: %s:%i:%s\n",
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].getMainMenuItem_StatusText()); */

    display.clearBuffer(); 
    display.setFont(u8g2_font_profont11_tr);
    display.setCursor(0,0);

    //display.print(crsf_devices[0].name);  
    Oled::printf(crsf_devices[0].name);
    display.setCursor(0,15);
    
    int menu_item_num = 0;
    int submenu_item_num = 0;
    for (size_t i = 0; i < crsf_devices[0].number_of_params-2 ; i++)
    {
        if (menuItems[i].parent==0) {
            menu_item_num++;

        } else submenu_item_num++;
       /*  db_out.printf("menu_item_num:%u:%u:%u\n",
                        menu_item_num,
                        submenu_item_num,
                        selected); */

    }
    
    // set line number that fits oled 128x64
    int num_lines = display.getRows()-3; 
    
    int start = (menu_item_num/num_lines)*num_lines;
    if (start > selected) start=start-num_lines;

    int pages = int(menu_item_num + num_lines -1) / num_lines;
 
    //debug main menu options
    #if !defined(DEBUG) 
        display.print("CRSF settings");  
    #else 
        display.print(selected); 
        display.print(":");
        display.print(pages); 
        display.print(":");
        display.print(start);
        display.print(":");
        display.print(menu_item_num);
        //db_out.printf("selected:%u:s:%u:num:%u\n",
        //selected,start,menu_item_num);
    #endif

    int i=start;
    int count=0;
    do
    {
        display.setFont(u8g2_font_profont11_tr);
           
        if ((menuItems[i].parent==0) && (menuItems[i].p_type !=12)) {
            count++;
            char *tmp = menuItems[i].name;
            display.setCursor(0,display.getCursorY()+9);
            if (i == selected) 
                display.print("> ");
          
          
            display.print(tmp);
            
            //text selection - items w/ menu
            if (menuItems[i].p_type==9) { 
                //int len = display.getDisplayWidth() - (strlen(menuItems[i].getMainMenuItem_StatusText())*6);
    
                //db_out.printf("len! %u;%u\n",len,  display.getDisplayWidth());
                
                Oled::PrintRight(menuItems[i].getMainMenuItem_StatusText());
                    
            }
            display.print("\n");  
        } 
        
        i++;

    } while (count < num_lines);

    display.sendBuffer();



}


void Oled::printf(char *tmp) {
    display.setCursor(0,display.getCursorY()+8);
    display.print(tmp);
}