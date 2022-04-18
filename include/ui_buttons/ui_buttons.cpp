void read_ui_buttons () {
    

    bool up = digitalRead(upBt);
    bool down = digitalRead(downBt);
    bool enter = digitalRead(enterBt);
    bool back = digitalRead(backBt);
    //TODO bt bouncer
    delay(250);
    //db_out.printf("%i%i%i%i\n",up,down,enter,back);
    //db_out.printf("%i:\n",params_loaded);

    if (up == LOW && down == LOW) {
    };
    
    //if at main screen
     if (entered==-2){ 
      //db_out.println("main");
      entered = enter ? entered : -1;

    //if inside main menu - selected = -1
    } else if (entered==-1){
      //db_out.println("menu");

      if ((up == LOW) && (entered != selected))
        //selected = (selected <= 0) ? 0 : selected-1;
      do {
        selected--;
        db_out.printf("select:%i:\n",selected);
        if (selected < 0) selected = crsf_devices[0].number_of_params-3;
      } while (menuItems[selected].parent != 0); 

      if ((down == LOW) && (entered != selected))
        //selected++;
      do {
        selected++;
        db_out.printf("select:%i:\n",selected);
        if (selected > crsf_devices[0].number_of_params-3) selected = 0;
      } while (menuItems[selected].parent != 0); 

      if ((enter == LOW)) {
        db_out.printf("click:%i:%i:%s:%u:%u\n",
        selected,
        menuItems[selected].parent,
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].max_value
        );

        if (menuItems[selected].p_type == 9) {
        db_out.printf("find:%i:%s:%u:%u\n",
        selected,
        menuItems[selected].name,
        menuItems[selected].status,
        menuItems[selected].max_value);
        //params_loaded = 0;
        

        if (menuItems[selected].status < menuItems[selected].max_value) {
          next_chunk = menuItems[selected].status + 1;
           } else next_chunk = 0;

        next_param = menuItems[selected].id;
        //next_chunk == cmd to send
        Menu::ChangeParam(next_param,next_chunk);
        

        } else {
            db_out.printf("not find:%i:%s\n",
                selected,
                menuItems[selected].name);
            subSelected = selected+1;
            entered = selected;
        }
      }
      if (back == LOW) entered = -2;

    //if at submenu
    } else if (entered>=0){
      //db_out.println("@submenu");

      subSelected=selected+1;
      if (enter == LOW) {

        db_out.printf("send cmd submenu \n %i:%i\n",
        selected,
        subSelected);
      }
      if (back == LOW) {
        entered = -1;
        subSelected = -1;
      }
      if ((up == LOW)) // && (entered != selected))
      {
        db_out.println("up");
        subSelected--;
      }
      if ((down == LOW)) // && (entered != subSelected))
      { 
        db_out.println("@down");
        subSelected++;
        }
  }


    //db_out.printf("ent:%i:sel:%isSel:%i\n",entered, selected,subSelected);
    
    //powerChangeHasRun=true;
    //clickCurrentMicros = crsfTime + (2*1000000);//2sec
    //delay(200);
}