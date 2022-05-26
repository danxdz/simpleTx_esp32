#pragma once
#include <Arduino.h>

class RxInfo
{
public:
    uint8_t id;
    uint8_t address;
    uint8_t num_params;
    uint32_t serial_number;
    char name[];
};

class RxParams
{

    uint8_t id;

public:
    char *name;
    char *get_rx_name();
    void setId(uint8_t RxId);
    uint8_t getId();
};

extern RxParams rx_p[];

extern uint8_t rx_params_loaded;