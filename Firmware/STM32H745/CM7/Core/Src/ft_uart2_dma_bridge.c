#include "ft_uart2_dma_bridge.h"
#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
/* Dedicated, 32-byte-aligned, exclusively DMA-owned storage. On H745
 * linker layouts this MUST be placed in DMA1-accessible SRAM (not DTCM).
 * The board memory map/MPU has not been physically commissioned. */
#define FT_DMA_BYTES 512U
#define FT_DMA_HALF (FT_DMA_BYTES/2U)
_Static_assert((FT_DMA_HALF%32U)==0U,"cache-line isolation");
/* Explicitly mapped to SRAM_D2 (DMA1 reachable); never DTCM. Both CM7
 * linker variants provide .dma_rx_d2 NOLOAD and 32-byte alignment. */
static _Alignas(32) uint8_t dma_bytes[FT_DMA_BYTES]
    __attribute__((section(".dma_rx_d2"), used));
static atomic_uint_least32_t finished_halves;
static atomic_uint_least32_t error_count;
static uint32_t observed_us[2];
static FtSerialPort *target;
static FtUart2NowUs clock_us;
static void *clock_context;
static uint32_t consumed_halves;
static uint32_t errors_seen;
static bool started;
static bool faulted;

bool ft_uart2_dma_start(FtSerialPort *port,FtUart2NowUs now,void *context)
{
    if(started||!port||!now||!port->protocol.commissioned ||
       huart2.Instance!=USART2||!huart2.hdmarx)
        return false;
    /* Preserve CubeMX clocks, GPIO, DMA request, UART baud/parity/RS485;
     * change only DMA circular ownership for uninterrupted reception. */
    if(HAL_DMA_DeInit(huart2.hdmarx)!=HAL_OK)return false;
    huart2.hdmarx->Init.Mode=DMA_CIRCULAR;
    if(HAL_DMA_Init(huart2.hdmarx)!=HAL_OK)return false;
    target=port;clock_us=now;clock_context=context;
    atomic_init(&finished_halves,0U);
    atomic_init(&error_count,0U);
    consumed_halves=0;errors_seen=0;faulted=false;
    /* Invalidate before first DMA transfer: no CPU owns this cache region. */
    SCB_InvalidateDCache_by_Addr((uint32_t*)dma_bytes,FT_DMA_BYTES);
    if(HAL_UART_Receive_DMA(&huart2,dma_bytes,FT_DMA_BYTES)!=HAL_OK){
        target=NULL;clock_us=NULL;return false;
    }
    started=true;return true;
}
static void completed(unsigned half)
{
    if(!started||!target||!clock_us)return;
    observed_us[half]=clock_us(clock_context);
    atomic_fetch_add_explicit(&finished_halves,1U,memory_order_release);
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart&&huart->Instance==USART2)completed(0U);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart&&huart->Instance==USART2)completed(1U);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart&&huart->Instance==USART2 && started)
        atomic_fetch_add_explicit(&error_count,1U,memory_order_release);
}
/* Call from a dedicated 1ms background service. HAL read/write waits must
 * never be added here or to the Supervisor. DMA1 callbacks only publish
 * completed-half indices and timestamps; task owns FtSerialPort producer. */
bool ft_uart2_dma_service(void)
{
    if(!started||!target||faulted)return false;
    uint32_t errors=atomic_load_explicit(&error_count,memory_order_acquire);
    if(errors!=errors_seen){
        errors_seen=errors;ft_serial_isr_uart_error(target);
        (void)ft_serial_poll(target,clock_us(clock_context),0U);
        faulted=true; return false; /* explicit restart only */
    }
    uint32_t available=atomic_load_explicit(&finished_halves,memory_order_acquire);
    uint32_t count=available-consumed_halves;
    if(count>1U){
        /* A delayed service cannot know that completed DMA data stayed
         * untouched. Drop it; never decode an overwritten half. */
        consumed_halves=available;
        ft_serial_isr_uart_error(target);
        (void)ft_serial_poll(target,clock_us(clock_context),0U);
        faulted=true; return false;
    }
    if(count==1U){
        uint32_t half=consumed_halves&1U;
        uint8_t *bytes=dma_bytes+half*FT_DMA_HALF;
        uint8_t snapshot[FT_DMA_HALF];
        const uint32_t received_us=observed_us[half];
        SCB_InvalidateDCache_by_Addr((uint32_t*)bytes,FT_DMA_HALF);
        memcpy(snapshot,bytes,sizeof(snapshot));
        /* The NEXT half completion means DMA has started overwriting this
         * half. Validate ownership before publishing ANY copied bytes. */
        if(atomic_load_explicit(&finished_halves,memory_order_acquire)!=available ||
           atomic_load_explicit(&error_count,memory_order_acquire)!=errors){
            ft_serial_isr_uart_error(target);
            (void)ft_serial_poll(target,clock_us(clock_context),0U);
            faulted=true; return false;
        }
        for(unsigned pos=0;pos<FT_DMA_HALF;pos+=FT_SERIAL_ISR_MAX_BYTES){
            if(!ft_serial_isr_receive(target,snapshot+pos,FT_SERIAL_ISR_MAX_BYTES,
                                      received_us)){
                faulted=true;
                ft_serial_isr_uart_error(target);
                return false;
            }
        }
        consumed_halves++;
        if(atomic_load_explicit(&finished_halves,memory_order_acquire)
            -consumed_halves>1U){
            ft_serial_isr_uart_error(target);faulted=true;return false;
        }
    }
    (void)ft_serial_poll(target,clock_us(clock_context),128U);
    return true;
}

/* Must only be called from the owning CM7 background task, never an ISR.
 * Stops the UART DMA before clearing the volatile ownership metadata. */
void ft_uart2_dma_stop(void)
{
    if (started) (void)HAL_UART_DMAStop(&huart2);
    started=false;
    faulted=false;
    target=NULL;
    clock_us=NULL;
    clock_context=NULL;
}
