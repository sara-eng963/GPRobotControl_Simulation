
#ifndef HANDGUIDING_ADMITTANCE_CONTROL_H
#define HANDGUIDING_ADMITTANCE_CONTROL_H

#include <stdbool.h>

#define ADMITTANCE_AXES 6U

/*
 * Axes 0..2: X, Y, Z translation.
 * Axes 3..5: X, Y, Z rotation.
 *
 * Mass: kg (translation), kg*m^2 (rotation).
 * Damping: N*s/m, N*m*s/rad.
 * Speed limits: m/s, rad/s.
 */
typedef struct {
    double virtual_mass[ADMITTANCE_AXES];
    double damping[ADMITTANCE_AXES];
    double speed_limit[ADMITTANCE_AXES];
} AdmittanceParams;

typedef struct {
    /* Cartesian twist [vx, vy, vz, wx, wy, wz]. */
    double velocity[ADMITTANCE_AXES];
} AdmittanceState;

bool admittance_params_valid(const AdmittanceParams *params);

void admittance_reset(AdmittanceState *state);

/*
 * Input: validated, compensated and transformed wrench
 *        [Fx, Fy, Fz, Tx, Ty, Tz] at the controlled point.
 *
 * Input wrench and output twist use the same fixed frame.
 *
 * Returns false on invalid inputs and resets state/output.
 */
bool admittance_step(
    const AdmittanceParams *params,
    AdmittanceState *state,
    const double wrench[ADMITTANCE_AXES],
    double period_s,
    double twist_out[ADMITTANCE_AXES]
);

#endif
