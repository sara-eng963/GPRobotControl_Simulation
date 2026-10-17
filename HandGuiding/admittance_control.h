#ifndef HANDGUIDING_ADMITTANCE_CONTROL_H
#define HANDGUIDING_ADMITTANCE_CONTROL_H

#include <stdbool.h>

#define ADMITTANCE_AXES 6U

/* All wrench/twist values refer to the SAME control point and fixed frame.
 * Indices 0..2: linear [N, m/s, m/s^2]; 3..5: angular [N*m, rad/s, rad/s^2].
 * Virtual mass units: kg (linear), kg*m^2 (angular).
 * Damping units: N*s/m (linear), N*m*s/rad (angular).
 * maximum_step_s is a caller-approved timing/fault threshold, not a default.
 */
typedef struct {
    double virtual_mass[ADMITTANCE_AXES];
    double damping[ADMITTANCE_AXES];
    double speed_limit[ADMITTANCE_AXES];
    double acceleration_limit[ADMITTANCE_AXES];
    double maximum_step_s;
} AdmittanceParams;

typedef struct {
    double velocity[ADMITTANCE_AXES];
    bool enabled;
} AdmittanceState;

bool admittance_params_valid(const AdmittanceParams *params);

/* Reset/disable always clears the previous velocity state. */
void admittance_reset(AdmittanceState *state);
void admittance_disable(AdmittanceState *state);

/* Enabling always starts from zero virtual velocity. It does NOT command
 * a robot pose: later motion integration must initialize joint references
 * independently from fresh measured drive positions. */
bool admittance_enable(AdmittanceState *state, const AdmittanceParams *params);

/* Backward-Euler mass-damper update, followed by per-axis bounded change
 * in velocity and velocity saturation. Acceleration limits and saturation
 * modify the ideal admittance dynamics; no interaction-passivity guarantee.
 * Returns false and disables/clears the controller on invalid data, timing
 * faults or while disabled. Output is zeroed on failure when non-NULL.
 * Zero output is NOT a physical safe-stop command or sensor fault policy.
 * twist_out must not overlap wrench or state storage.
 */
bool admittance_step(
    const AdmittanceParams *params,
    AdmittanceState *state,
    const double wrench[ADMITTANCE_AXES],
    double period_s,
    double twist_out[ADMITTANCE_AXES]
);

#endif /* HANDGUIDING_ADMITTANCE_CONTROL_H */
