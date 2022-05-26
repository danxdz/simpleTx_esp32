#include "rx_params.h"
#include "uart.h"
#include "crsf.h"

RxParams rx_p[CRSF_MAX_PARAMS];
uint8_t rx_params_loaded = 0;

uint8_t id = 0;
uint8_t address = 0;
uint8_t num_params = 0;
uint32_t serial_number = 0;
char name[CRSF_MAX_NAME_LEN];

char *RxParams::get_rx_name()
{
    dbout.printf("get_rx_p\n");
    return name;
}

void RxParams::setId(uint8_t RxId)
{
    id = RxId;
}

uint8_t RxParams::getId()
{
    return id;
}
