#include "dwin_port_stm32.h"

/* NOTE: This file does not include a specific stm32XX_hal_uart.h on purpose.
 * Include your CubeMX-generated HAL headers from your main.c or build system.
 */

/* Weak-ish fallbacks (avoid compile errors if headers not yet pulled) */
#ifndef HAL_OK
#define HAL_OK 0x00U
#endif

/* HAL prototypes (provided by Cube HAL) */
extern uint32_t HAL_GetTick(void);
extern int HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern int HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Tiny registry so ISR can find the port by huart */
#ifndef DWIN_STM32_MAX_PORTS
#define DWIN_STM32_MAX_PORTS 2
#endif
static DwinStm32Port *s_ports[DWIN_STM32_MAX_PORTS] = {0};

static void reg_port(DwinStm32Port *p)
{
    for (unsigned i = 0; i < DWIN_STM32_MAX_PORTS; ++i)
    {
        if (!s_ports[i])
        {
            s_ports[i] = p;
            return;
        }
    }
}
static DwinStm32Port *find_port(UART_HandleTypeDef *huart)
{
    for (unsigned i = 0; i < DWIN_STM32_MAX_PORTS; ++i)
        if (s_ports[i] && s_ports[i]->huart == huart)
            return s_ports[i];
    return NULL;
}

void dwin_stm32_attach(DwinStm32Port *port, Dwin *ctx, UART_HandleTypeDef *huart)
{
    if (!port)
        return;
    port->huart = huart;
    port->ctx = ctx;
    port->rx_byte = 0;
    reg_port(port);
}

void dwin_stm32_begin(DwinStm32Port *port)
{
    if (!port || !port->huart)
        return;
    /* Arm single-byte interrupt receive */
    HAL_UART_Receive_IT(port->huart, (uint8_t *)&port->rx_byte, 1);
}

size_t dwin_stm32_tx(DwinStm32Port *port, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (!port || !port->huart || !data || !len)
        return 0;
    if (HAL_UART_Transmit(port->huart, (const uint8_t *)data, (uint16_t)len,
                          timeout_ms ? timeout_ms : 1000u) == HAL_OK)
        return len;
    return 0;
}

/* Call this from HAL_UART_RxCpltCallback(huart). */
void dwin_stm32_rx_byte_isr(UART_HandleTypeDef *huart)
{
    DwinStm32Port *p = find_port(huart);
    if (!p || !p->ctx)
    {
        if (huart)
            HAL_UART_Receive_IT(huart, (uint8_t *)&p->rx_byte, 1);
        return;
    }
    extern void dwin_isr_feed(Dwin * ctx, uint8_t byte); /* declared in dwin.h */
    dwin_isr_feed(p->ctx, (uint8_t)p->rx_byte);
    HAL_UART_Receive_IT(huart, (uint8_t *)&p->rx_byte, 1); /* re-arm */
}