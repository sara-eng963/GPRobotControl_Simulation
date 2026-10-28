#ifndef VIRTUAL_FT_CONTROLS_H
#define VIRTUAL_FT_CONTROLS_H

#include <stdbool.h>

#define VIRTUAL_FT_AXIS_COUNT 6U

/* Axis order matches protocol: Fx,Fy,Fz,Tx,Ty,Tz. Opposing keys cancel. */
typedef struct {
    bool positive[VIRTUAL_FT_AXIS_COUNT];
    bool negative[VIRTUAL_FT_AXIS_COUNT];
} VirtualFtKeys;

/* Focus loss / zero button inhibit held inputs until every key is released.
 * This gate controls the simulated input only, never the Supervisor. */
typedef struct {
    bool suppressed_until_release;
} VirtualFtInputGate;

void virtual_ft_input_gate_update(VirtualFtInputGate *gate,
                                     bool focused, bool clear_pressed,
                                     const VirtualFtKeys *keys);

void virtual_ft_wrench_request(const VirtualFtKeys *keys,
                                  bool enabled, double force_N,
                                  double torque_Nm, double wrench[6]);

/* Retained for existing X-only clients/tests. */
double virtual_ft_x_request(bool plus_held, bool minus_held,
                               bool controls_enabled, double magnitude_N);
#endif
