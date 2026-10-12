#include "trajectory_stream_path_port.h"
#include <string.h>

_Static_assert(sizeof(PvExecutionSample) == sizeof(GpTrajectorySample),
               "Execution sample layout changed");

bool gp_traj_path_read_sample(uint32_t sample_index,
                              PvExecutionSample *sample, void *context)
{
    GpTrajectoryPathPort *port = context;
    if (port == NULL || port->consumer == NULL || sample == NULL)
        return false;
    GpTrajectorySample shared_sample;
    if (gp_traj_m7_take(port->consumer, sample_index, &shared_sample) !=
        GP_TRAJ_READ_OK) return false;
    memcpy(sample, &shared_sample, sizeof(shared_sample));
    return true;
}

bool gp_traj_path_skip_samples(uint32_t first, uint32_t count,
                               void *context)
{
    GpTrajectoryPathPort *port = context;
    return port != NULL && port->consumer != NULL &&
        gp_traj_m7_skip(port->consumer, first, count) == GP_TRAJ_READ_OK;
}

bool gp_traj_path_stream_ready(uint32_t next_index, uint32_t count,
                               void *context)
{
    GpTrajectoryPathPort *port = context;
    if (port == NULL || port->consumer == NULL ||
        count != port->identity.sample_count) return false;
    GpTrajectoryConsumer *consumer = port->consumer;
    if (!consumer->active)
        return next_index == 0U &&
               gp_traj_m7_begin(consumer, &port->identity);
    /* Pause/resume retains ownership; never restart the ring or skip samples. */
    if (next_index != consumer->next_index ||
        gp_traj_current_epoch(consumer->shared) != consumer->epoch ||
        __atomic_load_n(&consumer->shared->read_error, __ATOMIC_ACQUIRE) != 0U)
        return false;
    return next_index == count || gp_traj_buffered(consumer->shared) > 0U;
}

void gp_traj_path_end_stream(void *context)
{
    GpTrajectoryPathPort *port = context;
    if (port != NULL && port->consumer != NULL && port->consumer->active)
        (void)gp_traj_m7_finish(port->consumer);
}
