#ifndef GP_TRAJECTORY_STREAM_PATH_PORT_H
#define GP_TRAJECTORY_STREAM_PATH_PORT_H

#include "trajectory_stream.h"
#include "../StateMachine/States/state_path_execution.h"

/* M7-owned adapter: drop-in PathExecutionServices callback signatures.
 * No flash access, heap use, mutex, FreeRTOS dependency or waits on M7.
 */
typedef struct {
    GpTrajectoryConsumer *consumer;
    GpTrajectoryIdentity identity;
} GpTrajectoryPathPort;

bool gp_traj_path_read_sample(uint32_t sample_index,
                              PvExecutionSample *sample, void *context);
bool gp_traj_path_stream_ready(uint32_t next_index, uint32_t count,
                               void *context);
bool gp_traj_path_skip_samples(uint32_t first, uint32_t count,
                               void *context);
void gp_traj_path_end_stream(void *context);

/* B1 production service composition. Physical welding/stop/retraction
 * callbacks retain their original context; only the path samples come from
 * the M7-owned SRAM4 consumer. The validated artifact stays M7-private.
 * Allocate the binding in persistent M7 RAM (not the boot task stack). */
typedef struct {
    GpTrajectoryConsumer consumer;
    GpTrajectoryPathPort stream;
    const ValidatedTrajectory *artifact;
    PathExecutionServices physical;
    PathExecutionServices services;
} GpTrajectoryExecutionBinding;

/* Rejects duplicate stream providers or absent physical stop/relay hooks.
 * No prepared stream or hardware readiness is synthesized here. */
bool gp_traj_execution_bind(GpTrajectoryExecutionBinding *binding,
                            GpTrajectoryShared *shared,
                            const ValidatedTrajectory *artifact,
                            const PathExecutionServices *physical);

#endif
