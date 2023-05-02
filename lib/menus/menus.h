#ifndef MENUS_H /* include guards */
#define MENUS_H

#pragma once

#include "crsf.h"

enum data_type
{
    UINT8 = 0,
    INT8 = 1,
    UINT16 = 2,
    INT16 = 3,
    FLOAT = 8,
    TEXT_SELECTION = 9,
    STRING = 10,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
    OUT_OF_RANGE = 127,
};

extern int mmOptionSelected;

extern uint8_t params_loaded; // if not zero, number received so far for current device
extern uint8_t next_param;    // parameter and chunk currently being read
extern uint8_t next_chunk;

// setup menus
extern int entered; //-2 idle // -1 main menu // 0 options/submenu
extern int selected;
extern int subSelected;

class Menu
{

    uint8_t hidden;
    char *info;
    uint8_t timeout;
    uint8_t min_value;
    uint8_t count;
    uint8_t default_value;

public:
    uint8_t id;
    char *name;
    char *value;
    uint8_t parent;
    uint8_t p_type;
    uint8_t status;
    uint8_t max_value;
    char *optionsMainMenu[50];

    char *getMainMenuItem_StatusText();

    void displayInfo();

    void divideValueParam(char *values);

    void getParams(char *buffer, int iid);

    uint8_t getCount() const { return count; }
    char* getValue() const { return value; }

};

extern Menu menuItems[];

#endif