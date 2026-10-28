#include "admittance_control.h"

#include <math.h>
#include <stddef.h>

bool admittance_params_valid(const AdmittanceParams *params)
{
    if (params == NULL || !isfinite(params->maximum_step_s) ||
        params->maximum_step_s <= 0.0) {
        return false;
    }

    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        if (!isfinite(params->virtual_mass[i]) ||
            params->virtual_mass[i] <= 0.0 ||
            !isfinite(params->damping[i]) ||
            params->damping[i] <= 0.0 ||
            !isfinite(params->speed_limit[i]) ||
            params->speed_limit[i] <= 0.0 ||
            !isfinite(params->acceleration_limit[i]) ||
            params->acceleration_limit[i] <= 0.0) {
            return false;
        }
    }
    return true;
}

void admittance_reset(AdmittanceState *state)
{
    if (state == NULL) return;
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        state->velocity[i] = 0.0;
        state->previous_wrench[i] = 0.0;
    }
    state->enabled = false;
}

void admittance_disable(AdmittanceState *state)
{
    admittance_reset(state);
}

bool admittance_enable(AdmittanceState *state, const AdmittanceParams *params)
{
    if (state == NULL) return false;
    admittance_reset(state);
    if (!admittance_params_valid(params)) return false;
    state->enabled = true;
    return true;
}

bool admittance_step(
    const AdmittanceParams *params,
    AdmittanceState *state,
    const double wrench[ADMITTANCE_AXES],
    double period_s,
    double twist_out[ADMITTANCE_AXES])
{
    if (twist_out != NULL) {
        for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) twist_out[i] = 0.0;
    }
    if (state == NULL || !state->enabled || wrench == NULL ||
        twist_out == NULL || !admittance_params_valid(params) ||
        !isfinite(period_s) || period_s <= 0.0 ||
        period_s > params->maximum_step_s) {
        admittance_reset(state);
        return false;
    }

    double next[ADMITTANCE_AXES];
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        const double previous = state->velocity[i];
        if (!isfinite(previous) || !isfinite(wrench[i]) ||
            !isfinite(state->previous_wrench[i]) ||
            fabs(previous) > params->speed_limit[i]) {
            admittance_reset(state);
            return false;
        }

        const double mass = params->virtual_mass[i];
        const double damping = params->damping[i];
        /* Tustin: (2M - D*dt)*v_prev + dt*(w_prev + w_now). */
        const double denominator = 2.0 * mass + period_s * damping;
        const double numerator = (2.0 * mass - period_s * damping) * previous +
                                 period_s * (state->previous_wrench[i] + wrench[i]);
        const double dv_max = period_s * params->acceleration_limit[i];
        if (!isfinite(denominator) || denominator <= 0.0 ||
            !isfinite(numerator) || !isfinite(dv_max) || dv_max <= 0.0) {
            admittance_reset(state);
            return false;
        }
        const double ideal = numerator / denominator;
        if (!isfinite(ideal)) {
            admittance_reset(state);
            return false;
        }

        /* First bound delta velocity, then clamp final speed. Because the
         * previous speed is already in range, both constraints hold. */
        double delta = ideal - previous;
        if (!isfinite(delta)) {
            admittance_reset(state);
            return false;
        }
        if (delta > dv_max) delta = dv_max;
        else if (delta < -dv_max) delta = -dv_max;

        double candidate = previous + delta;
        if (!isfinite(candidate)) {
            admittance_reset(state);
            return false;
        }
        const double limit = params->speed_limit[i];
        if (candidate > limit) candidate = limit;
        else if (candidate < -limit) candidate = -limit;
        next[i] = candidate;
    }

    /* No axis can modify the controller state before all six have passed. */
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        state->velocity[i] = next[i];
        state->previous_wrench[i] = wrench[i];
        twist_out[i] = next[i];
    }
    return true;
}
