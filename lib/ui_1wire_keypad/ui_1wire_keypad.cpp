#include "ui_1wire_keypad.h"
#include "uart.h"
#include "menus.h"
#include "config.h"

void Keypad::init()
{
#if !defined(DEBUG)
    dbout.printf("starting keypad...\n");
#endif

    delay(500);
}

int currentMenuPosition = 0;

//no button pressed = 0
//center = 1
//left = 2
//up = 3
//right = 4
//down = 5


void Keypad::read_keypad(UI_input_t* UI_input) {

  if (UI_input->key == 0) {
    return;
  }

  dbout.printf("key:%i:%i:%i:%i:%i\n", UI_input->key, entered, selected, subSelected, mmOptionSelected);
  switch (entered) {
    case -2:
    //main
    dbout.printf("main screen\n");
    
       handle_main_screen(UI_input);
      break;

    case -1:
    dbout.printf("main menu\n");
      handle_main_menu(UI_input);
      break;

    case -10:
    dbout.printf("main menu option\n");
      handle_submenu(UI_input);
      break;

    default:
    dbout.printf("submenu\n");
      handle_menu_option_selection(UI_input);
      break;
  }
}
void Keypad::handle_main_screen(UI_input_t* UI_input) {
  if (UI_input->key == 2) {
    if (params_loaded < crsf_devices[0].number_of_params) {
      char* load = (char*)hdr_str_cb(menuItems);
      dbout.printf("hdr:%s - %d\n", load, params_loaded);
    } else {
      dbout.printf("all params\n");
      dbout.printf("start webserver\n");
      // startWebServer(params_loaded, menuItems);
    }
  }

  entered = (UI_input->key == 1) ? entered : -1;
}

void Keypad::handle_main_menu(UI_input_t* UI_input) {
  if (UI_input->key == 3 && entered != selected) {
    do {
      selected = (selected - 1 < 0) ? crsf_devices[0].number_of_params - 3 : selected - 1;
      dbout.printf("select -:%i:\n", selected);
    } while (menuItems[selected].parent != 0);
  }

  if (UI_input->key == 5 && entered != selected) {
    do {
      selected = (selected + 1 > crsf_devices[0].number_of_params - 3) ? 0 : selected + 1;
      dbout.printf("select +:%i:\n", selected);
    } while (menuItems[selected].parent != 0);
  }

  if (UI_input->key == 1) {
    if ((menuItems[selected].p_type == TEXT_SELECTION) || (menuItems[selected].p_type == COMMAND)) {
      dbout.printf("find:%i:%s:%u:%u\n", selected, menuItems[selected].name, menuItems[selected].status, menuItems[selected].max_value);
      entered = -10;
      mmOptionSelected = menuItems[selected].status;
    } else {
      dbout.printf("not find:%i:%s\n", selected, menuItems[selected].name);
      subSelected = selected + 1;
      entered = selected;
    }
  }

  if (UI_input->key == 2) {
    entered = -2;
  }
}

void Keypad::handle_submenu(UI_input_t* UI_input) {
  if (UI_input->key == 5) {
    mmOptionSelected = (mmOptionSelected < menuItems[selected].max_value) ? mmOptionSelected + 1 : 0;
  }

  if (UI_input->key == 3) {
    mmOptionSelected = (mmOptionSelected > 0) ? mmOptionSelected - 1 : menuItems[selected].max_value;
  }

  if (UI_input->key ==2) {
    dbout.printf("back to idle\n");
    entered = -1;
  }

  if (UI_input->key == 1) {
    dbout.printf("select option %u:%u\n", mmOptionSelected, selected);
    next_param = selected + 1;
    next_chunk = mmOptionSelected;

    dbout.printf("ChangeParam: %s:%u\n", menuItems[selected].name, menuItems[selected].p_type);
    if (menuItems[selected].p_type == COMMAND) {
      next_chunk = 4; // cmd
    }
    CRSF_changeParam(next_param, next_chunk);
  }
}

void Keypad::handle_menu_option_selection(UI_input_t* UI_input) {
  if (UI_input->key == 1) {
    next_param = menuItems[subSelected].id;

    if (menuItems[subSelected].status < menuItems[subSelected].max_value) {
      next_chunk = menuItems[subSelected].status + 1;
    } else {
      next_chunk = 0;
    }

    dbout.printf("send cmd submenu \n %i:%i:%u:%u:%u:type:%u:%u\n", selected, subSelected, menuItems[subSelected].id,
                 menuItems[subSelected].max_value, menuItems[subSelected].status, menuItems[subSelected].p_type, next_chunk);

    if (menuItems[subSelected].p_type == COMMAND) {
      next_chunk = 4; // cmd
    }

    dbout.printf("ChangeParam: %s::%s::%s\n", menuItems[selected].name, next_param, next_chunk);
    CRSF_changeParam(next_param, next_chunk);
  }

  if (UI_input->key == 2) {
    entered = -1;
    dbout.printf("back to main menu\n");
  }

  if (UI_input->key == 3) {
    subSelected--;
    dbout.printf("select -:%i:\n", subSelected);
  }

  if (UI_input->key == 5) {
    subSelected++;
    dbout.printf("select +:%i:\n", subSelected);
  }
}
