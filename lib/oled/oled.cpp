#include "Arduino.h"
#include "oled.h"
#include "xbmStrings.h"
#include "uart.h"
#include "crsf.h"
#include "menus.h"
#include "rx_params.h"

#define DEBUG


U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0,U8X8_PIN_NONE,22,21); //todo define pins


void Oled::PrintCenter(uint8_t y,char *tmp) {
    
    uint8_t x = (display.getDisplayWidth()/2) - (display.getStrWidth(tmp)/2);
    display.drawStr(x,y,tmp);
}
void Oled::PrintRight(char *tmp) {
    uint8_t x = display.getDisplayWidth() - display.getStrWidth(tmp);
    display.drawStr(x,display.getCursorY(),tmp);
}

//with y
void Oled::PrintRight(uint8_t y,char *tmp) {
    uint8_t x = display.getDisplayWidth() - (display.getStrWidth(tmp));
    display.drawStr(x,y,tmp);
}



void Oled::init() {
    #if defined(DEBUG) 
        dbout.printf("starting screen...\n"); 
    #endif

    display.begin();
    display.clearBuffer(); 
    display.setFont(u8g2_font_minicute_tr);
    Oled::PrintCenter(7,(char *)"simpleTX");

    display.drawXBM(48, 15, 32, 32, elrs32);  
    
    display.setFont(u8g2_font_5x7_mr);
    
    Oled::PrintCenter(58,(char *)"ExpressLRS");
    display.sendBuffer();
    delay(2000);
}


void Oled::setMainScreen(char *name, crsfLinkStatistics_t LinkStatistics,uint8_t bpkts,uint8_t gpkts) {
    //dbout.printf("Mainsreen\n");
    
    display.clearBuffer(); 
    display.setFont(u8g2_font_chikita_tr);

    if (name) {
    
    display.drawStr(0,7, name);
    display.setCursor(0,14);
    //display.print("tx ");
    char output[400];
    snprintf(output, sizeof output, "%u:%u | %idBm  \n", 
        LinkStatistics.rf_Mode,
        LinkStatistics.uplink_Link_quality,
        LinkStatistics.uplink_RSSI_1);
    display.print(output);
    
    
    display.drawStr(0,48,crsf_devices[1].name);


    snprintf(output, sizeof output, "%u:%u | %idBm", 
        LinkStatistics.rf_Mode,
        LinkStatistics.downlink_Link_quality,
        LinkStatistics.downlink_RSSI);
    display.drawStr(0,55,output);
    
    snprintf(output, sizeof output, "%u:%u",bpkts,gpkts);
    display.drawStr(0,64, output );

    display.setFont(u8g2_font_9x15_me);
    display.setCursor(90,10);    
    display.print(LinkStatistics.uplink_TX_Power+"mW");

    display.setFont(u8g2_font_10x20_mr);
    display.setCursor(0,35);    
    float vBat = 5;// (float)batteryVoltage.voltage/10;
    display.print(vBat,2);  
    display.setFont(u8g2_font_5x7_mr);
     
    } else {

    display.setFont(u8g2_font_9x15_me);

    display.setCursor(0,14);
    Oled::PrintCenter(25,(char*)"no tx");
    }

    display.sendBuffer();


}

void Oled::setSubMenuItems() {
    //dbout.printf("Subscreen\n");
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

    int submenu_item_num = 0;
    for (size_t i = 0; menuItems[i].name ; i++) {
        if (menuItems[i].parent == menuItems[selected].id )
            submenu_item_num++;
    }

    if (subSelected>submenu_item_num+selected) subSelected = selected+1;
    if (subSelected<selected+1) subSelected = selected+submenu_item_num;

    for (size_t i = 0; i<crsf_devices[0].number_of_params; i++) {
        if (menuItems[i].parent == menuItems[selected].id ) {
            //dbout.printf("%s:%u : \n ",menuItems[i].name,menuItems[i].id); 
            
            
            display.setCursor(0,display.getCursorY()+9);
            if (i == subSelected) 
                display.print("> ");
         
            display.print(menuItems[i].name);
            //dbout.printf("len! %u;%u\n",len,  display.getDisplayWidth());
            //display.setFont(u8g2_font_profont10_mr);
            if (menuItems[i].optionsMainMenu[menuItems[i].status])
                Oled::PrintRight(menuItems[i].optionsMainMenu[menuItems[i].status]);
            
        }
    }
 

    display.sendBuffer();
}


void Oled::setMainMenuItems() {
    /*  dbout.printf("Mainsreen: %s:%i:%s\n",
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].getMainMenuItem_StatusText()); 
  */
    display.clearBuffer(); 
    display.setFont(u8g2_font_profont11_tr);
    display.drawStr(0,7,crsf_devices[0].name);
    
    display.setCursor(0,15);
    
    int menu_item_num = 0;
    int submenu_item_num = 0;
    for (size_t i = 0; i < crsf_devices[0].number_of_params-2 ; i++)
    {
        if (menuItems[i].parent==0) {
            menu_item_num++;

        } else submenu_item_num++;
       /*  dbout.printf("menu_item_num:%u:%u:%u\n",
                        menu_item_num,
                        submenu_item_num,
                        selected); */

    }
    
    // set line number that fits oled 128x64
    int num_lines = display.getRows()-3; 
    
    int start = (menu_item_num/num_lines)*num_lines;
    if (start > selected) start=start-num_lines;

    //debug main menu options
    #if !defined(DEBUG) 
        display.print("CRSF settings");  
    #else 
        display.print(selected); 
        display.print(":");
        display.print(start);
        display.print(":");
        display.print(menu_item_num);
        //dbout.printf("selected:%u:s:%u:num:%u\n",
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
                //dbout.printf("len! %u;%u\n",len,  display.getDisplayWidth());
                Oled::PrintRight(menuItems[i].getMainMenuItem_StatusText());
            }
            display.print("\n");  
        } 
        i++;
    } while (count < num_lines);
    display.sendBuffer();
}

void Oled::println(char *tmp) {
    display.setCursor(0,display.getCursorY()+8);
    display.print(tmp);
}


void Oled::selectOptionMainMenu() {
    //db_out.printf("selectOptionMainMenu\n");
      /* db_out.printf("Mainsreen: %s:%i:%s\n",
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].getMainMenuItem_StatusText()); 
 */
    display.clearBuffer(); 
    display.setFont(u8g2_font_profont11_tr);
    display.drawStr(0,7,crsf_devices[0].name);

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

    //debug main menu options
    #if !defined(DEBUG) 
        display.print("CRSF settings");  
    #else 
        display.print(selected); 
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



            display.print(tmp);

            //text selection - items w/ menu
            if (menuItems[i].p_type==9) { 
                //int len = display.getDisplayWidth() - (strlen(menuItems[i].getMainMenuItem_StatusText())*6);
                //db_out.printf("selected: %i;%i\n",selected,i);
                if (i == selected) {
                   char output[20];

                    if (menuItems[i].optionsMainMenu[mmOptionSelected]){
                        snprintf(output, sizeof output, "<%s>",
                            menuItems[i].optionsMainMenu[mmOptionSelected]);
                        //  menuItems[i].getMainMenuItem_StatusText());
                        Oled::PrintRight(output); 
                    }
                } else Oled::PrintRight(menuItems[i].getMainMenuItem_StatusText());
            }
            display.print("\n");  
        } 
        i++;
    } while (count < num_lines);
    display.sendBuffer();
}
