#include "halfduplex.h"
#include "uart.h"

void ICACHE_RAM_ATTR duplex_set_RX()
{
#ifdef DEBUG_HALF_DUPLEX
  db_out.printf("rx: %u\n", micros());
#endif
  // dbout.printf("set rx\n");

  portDISABLE_INTERRUPTS();
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_MODE_INPUT));
#if GPIO_PIN_RCSIGNAL_UART_INV
  gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, U1RXD_IN_IDX, true);
  gpio_pulldown_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
  gpio_pullup_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
#else
  gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, U1RXD_IN_IDX, false);
  gpio_pullup_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
  gpio_pulldown_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
#endif
  portENABLE_INTERRUPTS();
}

void ICACHE_RAM_ATTR duplex_set_TX()
{
#ifdef DEBUG_HALF_DUPLEX
  db_out.printf("tx: %u\n", micros());
#endif
  // dbout.printf("set tx\n");

  portDISABLE_INTERRUPTS();
  ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_FLOATING));
  ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_FLOATING));
#if GPIO_PIN_RCSIGNAL_UART_INV
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 0));
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT));
  constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30;             // routes 0 to matrix slot
  gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, false); // Disconnect RX from all pads
  gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, U1TXD_OUT_IDX, true, false);
#else
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 1));
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT));
  constexpr uint8_t MATRIX_DETACH_IN_HIGH = 0x38;             // routes 1 to matrix slot
  gpio_matrix_in(MATRIX_DETACH_IN_HIGH, U1RXD_IN_IDX, false); // Disconnect RX from all pads
  gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, U1TXD_OUT_IDX, false, false);
#endif
  portENABLE_INTERRUPTS();
}
