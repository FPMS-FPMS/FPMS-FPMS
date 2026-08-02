/*
 * uart_transport.h -- micro-XRCE-DDS custom transport over UART0.
 *
 * UART0 is the Type-C connector (via the board's CP2102 bridge, GPIO43/44)
 * and it is the ONLY link to the host. Two consequences:
 *
 *  1. The baud rate is set here, in code, not in menuconfig, because the
 *     host's micro-ros-agent.service pins `-b 921600` and the two must
 *     agree exactly.
 *  2. NOTHING ELSE MAY WRITE TO UART0. sdkconfig.defaults sets
 *     CONFIG_ESP_CONSOLE_NONE=y for this reason: a single stray ESP_LOGI
 *     would corrupt the XRCE framing. If you need printf debugging, move
 *     the console to UART1 (the LiDAR header, GPIO17/18) -- see README.
 */
#ifndef UART_TRANSPORT_H
#define UART_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "uxr/client/profile/transport/custom/custom_transport.h"

#define UROS_UART_BAUD  921600

bool   uros_uart_open(struct uxrCustomTransport *transport);
bool   uros_uart_close(struct uxrCustomTransport *transport);
size_t uros_uart_write(struct uxrCustomTransport *transport,
                       const uint8_t *buf, size_t len, uint8_t *err);
size_t uros_uart_read(struct uxrCustomTransport *transport,
                      uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif /* UART_TRANSPORT_H */
