#pragma once
#include <Arduino.h>
#include "crsf_protocol.h"

class RxInfo {
    public:
        uint8_t id;
        uint8_t address;
        uint8_t num_params;
        uint32_t serial_number;
        char name[];
};

class RxParams {
    public:
        uint8_t id;
        char *name;
        char * get_rx_name();
};

extern RxParams rx_p[];

extern uint8_t rx_params_loaded;