#include "gpio.h"
#include "menus.h"
#include "uart.h"


void read_ui_buttons () {
    

    bool up = digitalRead(upBt);
    bool down = digitalRead(downBt);
    bool enter = digitalRead(enterBt);
    bool back = digitalRead(backBt);
    //TODO bt bouncer
    delay(150);

   // dbout.printf("%i:%i::%i:%i\n",enter,back,up,down);
    //dbout.printf("%i:\n",params_loaded);
    

    if (up == LOW && down == LOW) {
    };
    
    //if at main screen
     if (entered==-2){ 
      //dbout.println("main");
      entered = enter ? entered : -1;

    //if inside main menu - selected = -1
    } else if (entered==-1){
      //dbout.println("menu");

      if ((up == LOW) && (entered != selected))
        //selected = (selected <= 0) ? 0 : selected-1;
      do {
        selected--;
        //dbout.printf("select:%i:\n",selected);
        if (selected < 0) selected = crsf_devices[0].number_of_params-3;
      } while (menuItems[selected].parent != 0); 

      if ((down == LOW) && (entered != selected))
        //selected++;
      do {
        selected++;
        //dbout.printf("select:%i:\n",selected);
        if (selected > crsf_devices[0].number_of_params-3) selected = 0;
      } while (menuItems[selected].parent != 0); 

      if ((enter == LOW)) {
        dbout.printf("click:%i:%i:%s:%u:%u\n",
        selected,
        menuItems[selected].parent,
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].max_value
        );

        if ((menuItems[selected].p_type == 9) || ((menuItems[selected].p_type == 13))) {
        dbout.printf("find:%i:%s:%u:%u\n",
        selected,
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].max_value);
        //params_loaded = 0;
        entered = -10;
        mmOptionSelected = menuItems[selected].status;
/* 
        if (menuItems[selected].status < menuItems[selected].max_value) {
          next_chunk = menuItems[selected].status + 1;
           } else next_chunk = 0;

        next_param = menuItems[selected].id;
        if (menuItems[selected].p_type == 13) next_chunk = 4; //cmd  */

        //next_chunk == cmd to send
       // Menu::ChangeParam(next_param,next_chunk);
        

        } else {
            dbout.printf("not find:%i:%s\n",
                selected,
                menuItems[selected].name);

            subSelected = selected+1;
            entered = selected;
        }
      }
      if (back == LOW) entered = -2;

    //if at submenu

    } else if (entered == -10) {
        //db_out.printf("select options: %i:%i:%i\n",mmOptionSelected,up,back);
        if (down == 1) {
          //entered = -1;
          if (mmOptionSelected < menuItems[selected].max_value)
            mmOptionSelected++;
          else
            mmOptionSelected = 0;
        }
        if (up == 1) // && (entered != selected))
        {
          //db_out.println("up");
          if (mmOptionSelected > 0) 
            mmOptionSelected--;
          else mmOptionSelected = menuItems[selected].max_value;
        }

      if (back == LOW) entered = -1;
      if (enter == LOW) {
        dbout.printf("select option %u:%u\n",mmOptionSelected,selected);
        next_param = selected+1;
        next_chunk = mmOptionSelected;
        
       
        dbout.printf("ChangeParam: %s\n",menuItems[selected].name);
        CRSF_changeParam(next_param,next_chunk);
        
        //Menu::ChangeParam(next_param,next_chunk);
      }





    } else if (entered>=0){
      //dbout.println("@submenu");

      if (enter == LOW) {
        
        next_param = menuItems[subSelected].id;
      
        if (menuItems[subSelected].status < menuItems[subSelected].max_value) {
          next_chunk = menuItems[subSelected].status + 1;
           } else next_chunk = 0;

        dbout.printf("send cmd submenu \n %i:%i:%u:%u:%u:type:%u:%u\n",
              selected,
              subSelected,
              menuItems[subSelected].id,
              menuItems[subSelected].max_value,
              menuItems[subSelected].status,
              menuItems[subSelected].p_type,
              next_chunk);
        
              
        if (menuItems[subSelected].p_type == 13) next_chunk = 4; //cmd 
        
          dbout.printf("ChangeParam: %s\n",menuItems[selected].name);
          CRSF_changeParam(next_param,next_chunk);
          //Menu::ChangeParam(next_param,next_chunk);
      }
      if (back == LOW) {
        entered = -1;
        //subSelected = -1;
      }
      if ((up == LOW)) // && (entered != selected))
      {
        //dbout.println("up");
        subSelected--;
      }
      if ((down == LOW)) // && (entered != subSelected))
      { 
        //dbout.println("@down");
        subSelected++;
        }
  }


    //dbout.printf("ent:%i:sel:%isSel:%i\n",entered, selected,subSelected);
    
    //powerChangeHasRun=true;
    //clickCurrentMicros = crsfTime + (2*1000000);//2sec
    //delay(200);
}