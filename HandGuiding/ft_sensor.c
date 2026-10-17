
#include "ft_sensor.h"

#include <math.h>
#include <stddef.h>

bool ft_sensor_sample_usable(
    const FtSensorSample *sample,
    uint32_t now_us,
    uint32_t max_age_us
)
{
    /* Reject missing, invalid or unsupported inputs. */
    if (sample == NULL || !sample->valid ||
        max_age_us == 0U ||
        max_age_us >= UINT32_C(0x80000000)) {
        return false;
    }

    /* All six measurements must be finite. */
    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(sample->force_N[i]) ||
            !isfinite(sample->torque_Nm[i])) {
            return false;
        }
    }

    /*
     * Unsigned subtraction handles 32-bit timer rollover,
     * assuming valid short measurement intervals.
     */
    const uint32_t age_us =
        now_us - sample->received_time_us;

    return age_us <= max_age_us;
}
