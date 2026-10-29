/* Optional, uncommissioned USART2 receive service. CM7 FreeRTOS task.
 * Never runs in the Supervisor 2ms task; never enables drives or guides
 * hardware. A real vendor-specific board provider is deliberately absent. */
#include "application_ft_service.h"
#include "ft_uart2_dma_bridge.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>

#define FT_SERVICE_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define FT_SERVICE_STACK_WORDS 512U
#define FT_SERVICE_PRIORITY (tskIDLE_PRIORITY + 1U)

/* Real board+sensor vendor integration owns this strong definition.
 * This link remains undefined until the vendor decoder is specified. */
extern bool board_supply_ft_serial_config(FtBoardSerialConfig *config)
    __attribute__((weak));

static FtSerialPort g_port;
static FtUart2NowUs g_now;
static void *g_clock_ctx;
static FtSensorSample g_published;
static bool g_published_valid;
static atomic_bool g_enabled;
static TaskHandle_t g_worker;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t g_worker_tcb;
static StackType_t g_worker_stack[FT_SERVICE_STACK_WORDS];
#endif

static void ft_service_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    if (!ft_uart2_dma_start(&g_port, g_now, g_clock_ctx)) {
        atomic_store(&g_enabled,false);
    }
    for (;;) {
        if (atomic_load(&g_enabled) && !ft_uart2_dma_service()) {
            /* No silent restart after UART/DMA fault or starvation. */
            ft_uart2_dma_stop();
            atomic_store(&g_enabled,false);
        }
        FtSensorSample sample={0};
        bool valid=atomic_load(&g_enabled) &&
            ft_serial_latest(&g_port,g_now(g_clock_ctx),&sample);
        taskENTER_CRITICAL();
        g_published=sample;
        g_published_valid=valid;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake,FT_SERVICE_PERIOD_TICKS);
    }
}

bool application_ft_service_start(void)
{
    if (g_worker!=NULL || FT_SERVICE_PERIOD_TICKS==0U ||
        board_supply_ft_serial_config==NULL) return false;
    FtBoardSerialConfig config={0};
    if (!board_supply_ft_serial_config(&config) ||
        !config.decoder.commissioned || config.monotonic_us==NULL ||
        !ft_serial_init(&g_port,&config.decoder,config.link_timeout_us,
                        config.fragment_timeout_us,config.max_sample_age_us))
        return false;
    g_now=config.monotonic_us;
    g_clock_ctx=config.clock_context;
    atomic_store(&g_enabled,true);
    memset(&g_published,0,sizeof(g_published));
    g_published_valid=false;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    g_worker=xTaskCreateStatic(ft_service_task,"FT_RX",FT_SERVICE_STACK_WORDS,
        NULL,FT_SERVICE_PRIORITY,g_worker_stack,&g_worker_tcb);
#else
    if (xTaskCreate(ft_service_task,"FT_RX",FT_SERVICE_STACK_WORDS,
                    NULL,FT_SERVICE_PRIORITY,&g_worker)!=pdPASS)
        g_worker=NULL;
#endif
    if (g_worker==NULL) atomic_store(&g_enabled,false);
    return g_worker!=NULL;
}

bool application_ft_service_read(void *context,FtSensorSample *sample)
{
    (void)context;
    if (sample==NULL) return false;
    memset(sample,0,sizeof(*sample));
    if (!atomic_load(&g_enabled)||g_now==NULL) return false;
    taskENTER_CRITICAL();
    const bool valid=g_published_valid;
    if (valid) *sample=g_published;
    taskEXIT_CRITICAL();
    /* Independently enforce stale measurement using the same clock. */
    return valid && ft_sensor_sample_usable(sample,g_now(g_clock_ctx),
                                          g_port.max_sample_age_us);
}
