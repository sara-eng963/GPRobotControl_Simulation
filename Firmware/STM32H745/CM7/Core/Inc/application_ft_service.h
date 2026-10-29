#ifndef APPLICATION_FT_SERVICE_H
#define APPLICATION_FT_SERVICE_H
/* CM7 optional UART2 background service, not part of TIM3 Supervisor.
 * No verified vendor protocol/board provider -> no task or DMA started. */
#include <stdbool.h>
#include "ft_uart2_dma_bridge.h"

typedef struct {
    FtSerialProtocol decoder; /* vendor-specific, never a host mock */
    FtUart2NowUs monotonic_us; /* CM7-local monotonic microsecond clock */
    void *clock_context;
    uint32_t link_timeout_us;
    uint32_t fragment_timeout_us;
    uint32_t max_sample_age_us;
} FtBoardSerialConfig;

/* Board-specific, specification-backed provider; no default definition. */
bool board_supply_ft_serial_config(FtBoardSerialConfig *config);
bool application_ft_service_start(void);
/* Bounded task-safe snapshot for an observation-only dry-run callback.
 * Does not grant authority or call CANopen. */
bool application_ft_service_read(void *context, FtSensorSample *sample);
#endif
