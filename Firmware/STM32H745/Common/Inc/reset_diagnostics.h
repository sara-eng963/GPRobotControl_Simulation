#ifndef RESET_DIAGNOSTICS_H
#define RESET_DIAGNOSTICS_H
/* STM32H745 RM0399, RCC_RSR (0x0D0), RCC_C1_RSR (0x130),
 * RCC_C2_RSR (0x190). RMVF is bit 16. Masks are for read-only diagnosis;
 * only the CM7 early-startup owner may clear flags after capture. */
#include <stdbool.h>
#include <stdint.h>

enum {
    RESET_FLAG_C1 = UINT32_C(1) << 17,
    RESET_FLAG_C2 = UINT32_C(1) << 18,
    RESET_FLAG_D1 = UINT32_C(1) << 19,
    RESET_FLAG_D2 = UINT32_C(1) << 20,
    RESET_FLAG_BOR = UINT32_C(1) << 21,
    RESET_FLAG_PIN = UINT32_C(1) << 22,
    RESET_FLAG_POR = UINT32_C(1) << 23,
    RESET_FLAG_SOFT1 = UINT32_C(1) << 24,
    RESET_FLAG_SOFT2 = UINT32_C(1) << 25,
    RESET_FLAG_IWDG1 = UINT32_C(1) << 26,
    RESET_FLAG_IWDG2 = UINT32_C(1) << 27,
    RESET_FLAG_WWDG1 = UINT32_C(1) << 28,
    RESET_FLAG_WWDG2 = UINT32_C(1) << 29,
    RESET_FLAG_LPWR1 = UINT32_C(1) << 30,
    RESET_FLAG_LPWR2 = UINT32_C(1) << 31
};
#define RESET_FLAG_ALL (UINT32_C(0xFFFE0000))
#define RESET_FLAG_RMVF (UINT32_C(1) << 16)

typedef enum {
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_POWER_ON,
    RESET_CAUSE_BROWNOUT,
    RESET_CAUSE_SOFTWARE,
    RESET_CAUSE_WATCHDOG,
    RESET_CAUSE_PIN,
    RESET_CAUSE_CPU_OR_DOMAIN,
    RESET_CAUSE_MULTIPLE
} ResetCause;

typedef struct {
    uint32_t raw_rsr;
    ResetCause cause;
    bool captured;
} ResetDiagnostics;

static inline ResetCause reset_diagnostics_classify(uint32_t raw)
{
    const uint32_t flags = raw & ~RESET_FLAG_RMVF;
    const uint32_t watchdog = RESET_FLAG_IWDG1 | RESET_FLAG_IWDG2 |
        RESET_FLAG_WWDG1 | RESET_FLAG_WWDG2;
    const uint32_t software = RESET_FLAG_SOFT1 | RESET_FLAG_SOFT2;
    if (flags == 0U || (flags & ~RESET_FLAG_ALL) != 0U)
        return RESET_CAUSE_UNKNOWN;
    /* These groups may include PIN/C1/C2/D1/D2 as secondary evidence.
     * POR/BOR may coexist on a normal power-on reset. */
    if ((flags & watchdog) != 0U)
        return ((flags & (RESET_FLAG_POR | RESET_FLAG_BOR | software)) != 0U)
            ? RESET_CAUSE_MULTIPLE : RESET_CAUSE_WATCHDOG;
    if ((flags & software) != 0U)
        return ((flags & (RESET_FLAG_POR | RESET_FLAG_BOR)) != 0U)
            ? RESET_CAUSE_MULTIPLE : RESET_CAUSE_SOFTWARE;
    if ((flags & RESET_FLAG_POR) != 0U)
        return RESET_CAUSE_POWER_ON;
    if ((flags & RESET_FLAG_BOR) != 0U)
        return RESET_CAUSE_BROWNOUT;
    if ((flags & (RESET_FLAG_LPWR1 | RESET_FLAG_LPWR2)) != 0U)
        return RESET_CAUSE_CPU_OR_DOMAIN;
    if ((flags & RESET_FLAG_PIN) != 0U)
        return RESET_CAUSE_PIN;
    if ((flags & (RESET_FLAG_C1 | RESET_FLAG_C2 |
                  RESET_FLAG_D1 | RESET_FLAG_D2)) != 0U)
        return RESET_CAUSE_CPU_OR_DOMAIN;
    return RESET_CAUSE_UNKNOWN;
}

static inline ResetDiagnostics reset_diagnostics_from_raw(uint32_t raw)
{
    const ResetDiagnostics snapshot = {
        .raw_rsr = raw,
        .cause = reset_diagnostics_classify(raw),
        .captured = true
    };
    return snapshot;
}
#endif
