#pragma once

#ifndef UI_1WIRE_KEYPAD_H /* include guards */
#define UI_1WIRE_KEYPAD_H

#include "ui_buttons.h"

class Keypad
{

public:
    void init();

    void read_keypad(UI_input_t *UI_input);

    void handle_main_screen(UI_input_t *UI_input);

    void handle_main_menu(UI_input_t *UI_input);

    void handle_submenu(UI_input_t *UI_input);

    void handle_menu_option_selection(UI_input_t *UI_input);
};

#endif