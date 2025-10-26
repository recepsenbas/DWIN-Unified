#ifndef DWIN_PORT_STM32_H
#define DWIN_PORT_STM32_H

/*
 * DWIN Unified — STM32 HAL port glue (interrupt-driven RX)
 * --------------------------------------------------------
 * Minimal adapter to connect a HAL UART to the DWIN core using
 * dwin_use_isr_queue(&ctx, 1). You only need to:
 *   1) Attach your UART handle and DWIN context
 *   2) Start single-byte RX interrupt
 *   3) Call dwin_stm32_rx_byte_isr() from HAL_UART_RxCpltCallback
 *   4) Pass a tx wrapper to dwin_init()
 */

#include <stddef.h>
#include <stdint.h>
#include "dwin.h"

/* Forward-declare HAL type to avoid hard-coding a specific family header. */
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UART_HandleTypeDef *huart;   /* bound HAL UART */
    Dwin *ctx;                   /* bound DWIN context */
    volatile uint8_t rx_byte;    /* 1-byte IT buffer */
} DwinStm32Port;

/* Bind the DWIN context and HAL UART handle */
void dwin_stm32_attach(DwinStm32Port *port, Dwin *ctx, UART_HandleTypeDef *huart);

/* Begin single-byte RX interrupt (HAL_UART_Receive_IT) */
void dwin_stm32_begin(DwinStm32Port *port);

/* TX helper (blocking). Suitable for small frames. */
size_t dwin_stm32_tx(DwinStm32Port *port, const uint8_t *data, size_t len, uint32_t timeout_ms);

/* ISR hook: call from HAL_UART_RxCpltCallback(huart) */
void dwin_stm32_rx_byte_isr(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* DWIN_PORT_STM32_H */