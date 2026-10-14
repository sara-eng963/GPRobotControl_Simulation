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

/* Compose existing PathExecution services without changing their ownership.
 * The path reader shares one callback context with weld/stop/retract helpers,
 * so we forward those helpers to the commissioned board context. */
static bool identity_matches(const GpTrajectoryExecutionBinding *binding)
{
    if (binding == NULL || binding->artifact == NULL) return false;
    const ValidatedTrajectory *a = binding->artifact;
    const GpTrajectoryIdentity *id = &binding->stream.identity;
    return id->program_id == a->program_id &&
           id->source_revision == a->source_revision &&
           id->artifact_crc == a->artifact_crc &&
           id->sample_count == a->sample_count &&
           id->sample_period_us == a->sample_period_us;
}

static bool execution_read(uint32_t index, PvExecutionSample *sample,
                           void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->consumer.active && identity_matches(b) &&
           gp_traj_path_read_sample(index, sample, &b->stream);
}

static bool execution_skip(uint32_t first, uint32_t count, void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->consumer.active && identity_matches(b) &&
           gp_traj_path_skip_samples(first, count, &b->stream);
}

static bool execution_ready(uint32_t index, uint32_t count, void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    if (b == NULL || b->artifact == NULL) return false;
    if (!b->consumer.active) {
        if (index != 0U || b->artifact->sample_count == 0U ||
            b->artifact->sample_period_us != GP_TRAJ_SAMPLE_PERIOD_US)
            return false;
        /* Only an M4-published epoch with this EXACT artifact identity
         * can open the ring. A revalidated path cannot inherit old samples. */
        b->stream.identity = (GpTrajectoryIdentity) {
            .program_id = b->artifact->program_id,
            .source_revision = b->artifact->source_revision,
            .artifact_crc = b->artifact->artifact_crc,
            .sample_count = b->artifact->sample_count,
            .sample_period_us = b->artifact->sample_period_us
        };
    }
    if (!identity_matches(b)) return false;
    if (b->playback != NULL && !b->consumer.active &&
        !b->playback_requested) {
        if (!gp_playback_m7_request(b->playback, b->consumer.shared,
                                    &b->stream.identity))
            return false;
        b->playback_requested = true;
    }
    return gp_traj_path_stream_ready(index, count, &b->stream);
}

static void execution_end(void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    if (b == NULL) return;
    if (b->consumer.active)
        gp_traj_path_end_stream(&b->stream);
    else
        /* Aborted before prefetch finished: release an unused epoch so M4
         * can acknowledge and safely reuse its producer ring. */
        (void)gp_traj_m7_discard(&b->consumer);
    if (b->playback != NULL && b->playback_requested)
        gp_playback_m7_cancel(b->playback);
    b->playback_requested = false;
}

static bool execution_wire(bool enabled, void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->physical.set_wire_feed_enabled != NULL &&
           b->physical.set_wire_feed_enabled(enabled, b->physical.context);
}

static bool execution_prepare_retraction(void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->physical.prepare_retraction != NULL &&
           b->physical.prepare_retraction(b->physical.context);
}

static StateStepResult execution_retraction(void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->physical.step_retraction != NULL
        ? b->physical.step_retraction(b->physical.context)
        : STATE_STEP_FAILED;
}

static bool execution_clearance(void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->physical.clearance_verified != NULL &&
           b->physical.clearance_verified(b->physical.context);
}

static bool execution_stop(bool *stopped, void *context)
{
    GpTrajectoryExecutionBinding *b = context;
    return b != NULL && b->physical.controlled_stop != NULL &&
           b->physical.controlled_stop(stopped, b->physical.context);
}

bool gp_traj_execution_bind(GpTrajectoryExecutionBinding *binding,
                            GpTrajectoryShared *shared,
                            const ValidatedTrajectory *artifact,
                            const PathExecutionServices *physical)
{
    if (binding == NULL || !gp_traj_shared_valid(shared) ||
        artifact == NULL || physical == NULL ||
        physical->read_sample != NULL || physical->skip_samples != NULL ||
        physical->stream_ready != NULL || physical->end_stream != NULL ||
        physical->set_wire_feed_enabled == NULL ||
        physical->prepare_retraction == NULL ||
        physical->step_retraction == NULL ||
        physical->clearance_verified == NULL ||
        physical->controlled_stop == NULL)
        return false;
    memset(binding, 0, sizeof(*binding));
    gp_traj_m7_init(&binding->consumer, shared);
    binding->stream.consumer = &binding->consumer;
    binding->artifact = artifact;
    binding->physical = *physical;
    binding->services.read_sample = execution_read;
    binding->services.skip_samples = execution_skip;
    binding->services.stream_ready = execution_ready;
    binding->services.end_stream = execution_end;
    binding->services.set_wire_feed_enabled = execution_wire;
    binding->services.prepare_retraction = execution_prepare_retraction;
    binding->services.step_retraction = execution_retraction;
    binding->services.clearance_verified = execution_clearance;
    binding->services.controlled_stop = execution_stop;
    binding->services.context = binding;
    binding->services.prefill_timeout_ms = physical->prefill_timeout_ms;
    return true;
}

bool gp_traj_execution_attach_playback(GpTrajectoryExecutionBinding *binding,
                                       GpValidationShared *validation)
{
    if (binding == NULL || binding->stream.consumer != &binding->consumer ||
        !gp_validation_shared_valid(validation) ||
        binding->physical.prefill_timeout_ms == 0U ||
        binding->playback != NULL)
        return false;
    binding->playback = validation;
    return true;
}
