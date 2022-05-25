#pragma once 

#include <Arduino.h>

extern bool powerChangeHasRun;


extern uint32_t tickTime;
extern uint16_t rates[];
//click deboucer
static uint32_t clickCurrentMicros = 0;



#define TEMPSTRINGLENGTH 400 //This is the max dialog size (80 characters * 5 lines)
                             //We could reduce this to ~240 on the 128x64 screens
                             //But only after all sprintf are replaced with snprintf
                             //Maybe move this to target_defs.h
extern char tempstring[TEMPSTRINGLENGTH];

void  check_link_state(uint32_t currentMicros);
const char * hdr_str_cb(const void *data);
void crsfdevice_init();
void bt_handle(uint8_t value);