#include "raylib.h"

/*
 * Keep the desktop HMI deliberately lightweight while the same WSL host is
 * running the 1 ms FreeRTOS/SOEM + KickCAT simulation.
 *
 * hmi_app.c still expresses its preferred desktop settings, but these linker
 * wrappers enforce simulation-safe runtime limits without coupling UI code to
 * EtherCAT timing concerns.
 */

#define HMI_SAFE_TARGET_FPS 20

void __real_SetTargetFPS(int fps);
void __real_SetConfigFlags(unsigned int flags);

void __wrap_SetTargetFPS(int fps)
{
    (void)fps;

    /*
     * The control loop remains 1 kHz.  Only the GUI redraw rate is reduced.
     */
    __real_SetTargetFPS(HMI_SAFE_TARGET_FPS);
}

void __wrap_SetConfigFlags(unsigned int flags)
{
    /*
     * 4x MSAA is unnecessary for an industrial HMI and adds GPU/WSLg load.
     * Preserve every other flag requested by the application.
     */
    flags &= ~FLAG_MSAA_4X_HINT;

    __real_SetConfigFlags(flags);
}
