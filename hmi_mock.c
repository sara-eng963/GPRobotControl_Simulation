#include "HMI-Mock/hmi_app.h"

#include <sys/resource.h>

int main(void)
{
    (void)setpriority(
        PRIO_PROCESS,
        0,
        15
    );

    return
        hmi_app_run();
}
