#ifndef GP_VIRTUAL_FT_CONTROLS_H
#define GP_VIRTUAL_FT_CONTROLS_H

#include <stdbool.h>

#define GP_VFT_AXIS_COUNT 6U

/* Axis order matches protocol: Fx,Fy,Fz,Tx,Ty,Tz. Opposing keys cancel. */
typedef struct {
    bool positive[GP_VFT_AXIS_COUNT];
    bool negative[GP_VFT_AXIS_COUNT];
} GpVirtualFtKeys;

/* Focus loss / zero button inhibit held inputs until every key is released.
 * This gate controls the simulated input only, never the Supervisor. */
typedef struct {
    bool suppressed_until_release;
} GpVirtualFtInputGate;

void gp_virtual_ft_input_gate_update(GpVirtualFtInputGate *gate,
                                     bool focused, bool clear_pressed,
                                     const GpVirtualFtKeys *keys);

void gp_virtual_ft_wrench_request(const GpVirtualFtKeys *keys,
                                  bool enabled, double force_N,
                                  double torque_Nm, double wrench[6]);

/* Retained for existing X-only clients/tests. */
double gp_virtual_ft_x_request(bool plus_held, bool minus_held,
                               bool controls_enabled, double magnitude_N);
#endif
