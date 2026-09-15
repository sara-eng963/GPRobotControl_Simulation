#include "HMI/hmi_app.h"

#include <sys/resource.h>

int main(void)
{
    /*
     * The HMI is soft real-time only.  Give FreeRTOS/SOEM and the KickCAT
     * simulator priority over GUI redraw work when everything runs in WSL.
     * Raising the nice value is permitted for an unprivileged process.
     */
    (void)setpriority(PRIO_PROCESS, 0, 15);

    return hmi_app_run();
}
