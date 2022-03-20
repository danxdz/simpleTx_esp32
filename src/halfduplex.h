#include <Arduino.h>

#define GPIO_PIN_RCSIGNAL_TX 13
#define GPIO_PIN_RCSIGNAL_RX 13

void duplex_set_RXs()
{
#ifdef debug
  db_out.printf("rx: %u",micros());
  db_out.println("");
#endif

  portDISABLE_INTERRUPTS();
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_MODE_INPUT));
    gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, U1RXD_IN_IDX, false);
    gpio_pullup_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    gpio_pulldown_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
  portENABLE_INTERRUPTS();
}

void duplex_set_TXs()
{
#ifdef debug
  db_out.printf("tx: %u",micros());
  db_out.println("");
#endif

  portDISABLE_INTERRUPTS();
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_FLOATING));

    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 1));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT));
    constexpr uint8_t MATRIX_DETACH_IN_HIGH = 0x38; // routes 1 to matrix slot
    gpio_matrix_in(MATRIX_DETACH_IN_HIGH, U1RXD_IN_IDX, false); // Disconnect RX from all pads
    gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, U1TXD_OUT_IDX, false, false);
  portENABLE_INTERRUPTS();
}
