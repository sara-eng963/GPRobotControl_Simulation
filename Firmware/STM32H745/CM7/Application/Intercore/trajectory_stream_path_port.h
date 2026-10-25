#ifndef TRAJECTORY_STREAM_PATH_PORT_H
#define TRAJECTORY_STREAM_PATH_PORT_H

#include "trajectory_stream.h"
#include "trajectory_storage_service.h"
#include "../StateMachine/States/state_path_execution.h"

/* M7-owned adapter: drop-in PathExecutionServices callback signatures.
 * No flash access, heap use, mutex, FreeRTOS dependency or waits on M7.
 */
typedef struct {
    TrajectoryConsumer *consumer;
    TrajectoryIdentity identity;
} TrajectoryPathPort;

bool trajectory_path_read_sample(uint32_t sample_index,
                              PvExecutionSample *sample, void *context);
bool trajectory_path_stream_ready(uint32_t next_index, uint32_t count,
                               void *context);
bool trajectory_path_skip_samples(uint32_t first, uint32_t count,
                               void *context);
void trajectory_path_end_stream(void *context);

/* B1 production service composition. Physical welding/stop/retraction
 * callbacks retain their original context; only the path samples come from
 * the M7-owned SRAM4 consumer. The validated artifact stays M7-private.
 * Allocate the binding in persistent M7 RAM (not the boot task stack). */
typedef struct {
    TrajectoryConsumer consumer;
    TrajectoryPathPort stream;
    const ValidatedTrajectory *artifact;
    ValidationShared *playback;
    bool playback_requested;
    PathExecutionServices physical;
    PathExecutionServices services;
} TrajectoryExecutionBinding;

/* Rejects duplicate stream providers or absent physical stop/relay hooks.
 * No prepared stream or hardware readiness is synthesized here. */
/* Enable an M7-only request channel to the M4 NOR reader. Must be called
 * after bind, before Supervisor starts. No M4 worker means fail closed. */
bool trajectory_execution_attach_playback(TrajectoryExecutionBinding *binding,
                                       ValidationShared *validation);

bool trajectory_execution_bind(TrajectoryExecutionBinding *binding,
                            TrajectoryShared *shared,
                            const ValidatedTrajectory *artifact,
                            const PathExecutionServices *physical);

#endif
