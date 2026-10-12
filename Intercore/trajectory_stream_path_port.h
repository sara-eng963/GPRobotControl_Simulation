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
void gp_traj_path_end_stream(void *context);

#endif
