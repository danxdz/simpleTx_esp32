#include <Arduino.h>

#define GPIO_PIN_RCSIGNAL_TX 13
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_UART_INV false

void ICACHE_RAM_ATTR duplex_set_RX();
void ICACHE_RAM_ATTR duplex_set_TX();
