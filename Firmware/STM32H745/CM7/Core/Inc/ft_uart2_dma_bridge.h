#ifndef FT_UART2_DMA_BRIDGE_H
#define FT_UART2_DMA_BRIDGE_H
/* Uncommissioned USART2 transport (PD4 DE, PD5 TX, PD6 RX).
 * Only a separately commissioned board service may start this receiver;
 * no source in this bridge can enable motors or authorize motion.
 * HAL callbacks: CM7 ISR; service: dedicated low-priority CM7 task, not
 * TIM3 Supervisor. Both callbacks and service use the same CM7 clock domain.
 */
#include "main.h"
#include "../../../../../HandGuiding/ft_serial_transport.h"
#include <stdbool.h>
#include <stdint.h>

typedef uint32_t (*FtUart2NowUs)(void *context);
/* Must be called after MX_DMA_Init and MX_USART2_UART_Init, and before
 * scheduling any service task. Returns false without changing drive state. */
bool ft_uart2_dma_start(FtSerialPort *port,FtUart2NowUs now,void *context);
/* One 256-byte completed half per service maximum, plus bounded parser
 * work of 128 bytes. Call no less often than every 1ms at 1 Mbaud 8N1.
 * A missed half is a transport fault, never a silently reused sample. */
bool ft_uart2_dma_service(void);
/* Explicit stop after error or shutdown; there is no automatic restart. */
void ft_uart2_dma_stop(void);
#endif
