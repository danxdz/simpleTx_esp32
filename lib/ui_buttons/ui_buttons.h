#pragma once 



typedef struct {
  int up = 0;
  int down = 0; 
  int enter = 0;
  int back = 0;
} UI_input_t;


void readUIbuttons (UI_input_t* UI_input);

void read_ui_buttons(UI_input_t* UI_input);
