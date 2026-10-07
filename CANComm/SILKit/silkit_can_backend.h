#ifndef SILKIT_CAN_BACKEND_H
#define SILKIT_CAN_BACKEND_H

#include "../can_backend.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const char *participant_name;
    const char *controller_name;
    const char *network_name;
    const char *registry_uri;

    uint32_t bitrate;

} SilKitCanBackendConfig;


/*
 * Create a SIL Kit implementation of the generic CanBackend.
 *
 * Example:
 *
 * participant_name = "RobotController"
 * controller_name  = "RobotCAN"
 * network_name     = "CAN1"
 * registry_uri     = "silkit://localhost:8500"
 * bitrate          = 1000000
 */
bool silkit_can_backend_create(
    CanBackend *backend,
    const SilKitCanBackendConfig *config
);

/*
 * Wait until SIL Kit has called CommunicationReady and the CAN controller
 * has been started. This is required before the CANopen Supervisor begins
 * BOOT traffic.
 */
bool silkit_can_backend_wait_ready(
    CanBackend *backend,
    uint32_t timeout_ms
);

/*
 * SIL Kit-specific observer receive helper.
 *
 * rx_time_ns is captured inside the SIL Kit frame callback using
 * std::chrono::steady_clock. This is intended for PC diagnostics such as the
 * live CAN monitor. The generic CanBackend API remains unchanged for
 * controller code.
 */
bool silkit_can_backend_receive_timestamped(
    CanBackend *backend,
    CanFrame *frame,
    uint64_t *rx_time_ns
);

#ifdef __cplusplus
}
#endif

#endif /* SILKIT_CAN_BACKEND_H */