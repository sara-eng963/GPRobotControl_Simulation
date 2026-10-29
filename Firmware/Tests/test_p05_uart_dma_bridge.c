/* Compile the actual production bridge .c with host HAL substitutes.
 * This tests ISR -> scheduled task ownership and failure containment;
 * HAL and MPU are NOT running on an STM32. */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "../../HandGuiding/ft_serial_transport.h"

/* Suppress the real HAL-only bridge header while including the unmodified
 * production translation unit; all substituted hardware types are explicit. */
#define FT_UART2_DMA_BRIDGE_H
#define USART2 ((void *)(uintptr_t)0x40004400U)
#define DMA_CIRCULAR 1U
#define HAL_OK 0
#define HAL_ERROR 1

typedef struct {unsigned Mode;} DMA_Init;
typedef struct {DMA_Init Init;} DMA_HandleTypeDef;
typedef struct {void *Instance;DMA_HandleTypeDef *hdmarx;} UART_HandleTypeDef;
typedef uint32_t (*FtUart2NowUs)(void *);
static DMA_HandleTypeDef test_dma;
UART_HandleTypeDef huart2={.Instance=USART2,.hdmarx=&test_dma};
static unsigned dma_stops,dma_starts;
static bool completion_during_copy;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static int HAL_DMA_DeInit(DMA_HandleTypeDef *d){(void)d;return HAL_OK;}
static int HAL_DMA_Init(DMA_HandleTypeDef *d){assert(d->Init.Mode==DMA_CIRCULAR);return HAL_OK;}
static int HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *bytes,unsigned n){
    assert(u==&huart2&&bytes!=NULL&&n==512U);dma_starts++;return HAL_OK;
}
static int HAL_UART_DMAStop(UART_HandleTypeDef *u){assert(u==&huart2);dma_stops++;return HAL_OK;}
static void SCB_InvalidateDCache_by_Addr(uint32_t *p,int32_t n){
    assert(((uintptr_t)p % 32U)==0U && (n%32)==0);
    if(completion_during_copy && n==256) {
        completion_during_copy=false;
        HAL_UART_RxCpltCallback(&huart2);
    }
}
#include "../STM32H745/CM7/Core/Src/ft_uart2_dma_bridge.c"
static uint32_t test_now=10000U;
static uint32_t tick(void *unused){(void)unused;return test_now;}
static uint32_t next_seq;
static void decoder_reset(void *ctx){(void)ctx;}
static bool decoder_partial(const void *ctx){(void)ctx;return false;}
static FtSerialDecodeResult decoder_feed(void *ctx,uint8_t b,FtSerialDecoded *out){
    (void)ctx;
    memset(out,0,sizeof(*out));
    out->sequence=++next_seq;
    out->force_N[0]=b;
    return FT_SERIAL_FRAME;
}
static FtSerialPort serial;
static void new_serial(bool commissioned){
    FtSerialProtocol p={.reset=decoder_reset,.partial=decoder_partial,
                        .feed=decoder_feed,.commissioned=commissioned};
    next_seq=0U;
    assert(ft_serial_init(&serial,&p,10000U,3000U,5000U));
}
int main(void){
    new_serial(false);
    assert(!ft_uart2_dma_start(&serial,tick,NULL));
    new_serial(true);
    assert(ft_uart2_dma_start(&serial,tick,NULL));
    memset(dma_bytes,7,FT_DMA_HALF);
    HAL_UART_RxHalfCpltCallback(&huart2);
    assert(ft_uart2_dma_service());
    assert(serial.diagnostics.frames_accepted==128U);
    test_now+=1000U;
    assert(ft_uart2_dma_service());
    assert(serial.diagnostics.frames_accepted==256U);
    FtSensorSample latest={0};
    assert(ft_serial_latest(&serial,test_now,&latest));
    assert(latest.force_N[0]==7.0);
    /* Lose the entire DMA ownership interval: do not accept overwritten data. */
    HAL_UART_RxCpltCallback(&huart2);
    HAL_UART_RxHalfCpltCallback(&huart2);
    assert(!ft_uart2_dma_service());
    assert(!ft_uart2_dma_service()); /* latched, no implicit recovery */
    ft_uart2_dma_stop();
    assert(dma_stops==1U);
    new_serial(true);
    assert(ft_uart2_dma_start(&serial,tick,NULL));
    HAL_UART_ErrorCallback(&huart2);
    assert(!ft_uart2_dma_service());
    assert(!ft_uart2_dma_service());
    ft_uart2_dma_stop();
    assert(dma_stops==2U && dma_starts==2U);
    /* Preempt the consumer at the DMA ownership boundary. No raced byte
     * reaches the decoder, and merely restoring time cannot restart RX. */
    new_serial(true);
    assert(ft_uart2_dma_start(&serial,tick,NULL));
    HAL_UART_RxHalfCpltCallback(&huart2);
    completion_during_copy=true;
    assert(!ft_uart2_dma_service());
    assert(serial.diagnostics.frames_accepted==0U);
    assert(!ft_uart2_dma_service());
    ft_uart2_dma_stop();
    assert(dma_stops==3U && dma_starts==3U);
    puts("PASS: production UART2 DMA bridge host ISR/task fault and restart contract");
    return 0;
}
