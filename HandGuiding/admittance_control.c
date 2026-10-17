
#include "admittance_control.h"

#include <math.h>
#include <stddef.h>

bool admittance_params_valid(const AdmittanceParams *params)
{
    if (params == NULL) {
        return false;
    }

    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        if (!isfinite(params->virtual_mass[i]) ||
            params->virtual_mass[i] <= 0.0 ||
            !isfinite(params->damping[i]) ||
            params->damping[i] <= 0.0 ||
            !isfinite(params->speed_limit[i]) ||
            params->speed_limit[i] <= 0.0) {
            return false;
        }
    }

    return true;
}

void admittance_reset(AdmittanceState *state)
{
    if (state == NULL) {
        return;
    }

    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        state->velocity[i] = 0.0;
    }
}

bool admittance_step(
    const AdmittanceParams *params,
    AdmittanceState *state,
    const double wrench[ADMITTANCE_AXES],
    double period_s,
    double twist_out[ADMITTANCE_AXES]
)
{
    /* Default output to zero on failure. */
    if (twist_out != NULL) {
        for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
            twist_out[i] = 0.0;
        }
    }

    if (state == NULL || wrench == NULL ||
        twist_out == NULL ||
        !admittance_params_valid(params) ||
        !isfinite(period_s) || period_s <= 0.0) {
        admittance_reset(state);
        return false;
    }

    double next[ADMITTANCE_AXES];

    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        if (!isfinite(wrench[i]) ||
            !isfinite(state->velocity[i])) {
            admittance_reset(state);
            return false;
        }

        const double m = params->virtual_mass[i];
        const double d = params->damping[i];

        /*
         * Backward Euler:
         *
         * v_next = (M*v + dt*W) / (M + dt*D)
         */
        const double denominator = m + period_s * d;
        const double numerator =
            m * state->velocity[i] +
            period_s * wrench[i];

        if (!isfinite(denominator) ||
            denominator <= 0.0 ||
            !isfinite(numerator)) {
            admittance_reset(state);
            return false;
        }

        next[i] = numerator / denominator;

        if (!isfinite(next[i])) {
            admittance_reset(state);
            return false;
        }

        /* Per-axis Cartesian velocity limit. */
        const double limit = params->speed_limit[i];

        if (next[i] > limit) {
            next[i] = limit;
        } else if (next[i] < -limit) {
            next[i] = -limit;
        }
    }

    /* Commit new state only after all axes are valid. */
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        state->velocity[i] = next[i];
        twist_out[i] = next[i];
    }

    return true;
}
