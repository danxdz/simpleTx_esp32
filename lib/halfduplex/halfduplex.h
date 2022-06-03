#include <Arduino.h>

#if defined(TARGET_ESP32)

  #define GPIO_PIN_RCSIGNAL_TX 13
  #define GPIO_PIN_RCSIGNAL_RX 13
  #define GPIO_PIN_RCSIGNAL_UART_INV false

#elif defined(TARGET_ESP32_S)

  #define GPIO_PIN_RCSIGNAL_TX 17
  #define GPIO_PIN_RCSIGNAL_RX 17
  #define GPIO_PIN_RCSIGNAL_UART_INV true

#endif

void ICACHE_RAM_ATTR duplex_set_RX();
void ICACHE_RAM_ATTR duplex_set_TX();
