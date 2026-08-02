#include "uart_transport.h"

#include "board_pins.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#define UROS_UART_PORT      UART_NUM_0
#define UROS_UART_RX_BUF    2048
#define UROS_UART_TX_BUF    2048

bool uros_uart_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    const uart_config_t cfg = {
        .baud_rate  = UROS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(UROS_UART_PORT, &cfg) != ESP_OK) {
        return false;
    }
    /* These are UART0's default pins on the ESP32-S3, so this call changes
     * nothing -- it is here so the wiring is stated in the source rather
     * than assumed. */
    if (uart_set_pin(UROS_UART_PORT, PIN_UART0_TX, PIN_UART0_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    if (!uart_is_driver_installed(UROS_UART_PORT)) {
        if (uart_driver_install(UROS_UART_PORT, UROS_UART_RX_BUF, UROS_UART_TX_BUF,
                                0, NULL, 0) != ESP_OK) {
            return false;
        }
    }
    (void)uart_flush_input(UROS_UART_PORT);
    return true;
}

bool uros_uart_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    if (uart_is_driver_installed(UROS_UART_PORT)) {
        (void)uart_driver_delete(UROS_UART_PORT);
    }
    return true;
}

size_t uros_uart_write(struct uxrCustomTransport *transport,
                       const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    const int written = uart_write_bytes(UROS_UART_PORT, (const char *)buf, len);
    if (written < 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)written;
}

size_t uros_uart_read(struct uxrCustomTransport *transport,
                      uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    if (timeout < 0) {
        timeout = 0;
    }
    const int got = uart_read_bytes(UROS_UART_PORT, buf, len, pdMS_TO_TICKS(timeout));
    if (got < 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)got;
}
