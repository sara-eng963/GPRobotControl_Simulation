#ifndef GP_VIRTUAL_FT_PROTOCOL_H
#define GP_VIRTUAL_FT_PROTOCOL_H

/* Host simulation ONLY. Atomic datagram: 10 big-endian 32-bit words.
 * [magic,version,sequence,enable,Fx,Fy,Fz,Tx,Ty,Tz].
 * The last six words are IEEE-754 float32 bit patterns, base-frame wrench
 * ABOUT THE TCP, not joint or Cartesian position commands. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GP_VFT_PORT 5013U
#define GP_VFT_MAGIC UINT32_C(0x56465431) /* VFT1 */
#define GP_VFT_VERSION 1U
#define GP_VFT_PACKET_SIZE 40U
#define GP_VFT_DEFAULT_TIMEOUT_US 150000U

typedef struct {
    double max_force_N;
    double max_torque_Nm;
    uint32_t timeout_us;
} GpVirtualFtLimits;

typedef enum {
    GP_VFT_FAULT_NONE = 0,
    GP_VFT_FAULT_INVALID_PACKET,
    GP_VFT_FAULT_SEQUENCE,
    GP_VFT_FAULT_TIMEOUT
} GpVirtualFtFault;

typedef struct {
    double wrench_base_tcp[6];
    uint32_t sequence;
    uint32_t received_us;
    bool connected;
    bool enabled;
    bool fault_latched;
    GpVirtualFtFault fault_reason;
} GpVirtualFtState;

/* Serialize the same packet format the existing receiver validates.
 * Does not impose sensor limits: receiver owns the authoritative limits. */
bool gp_vft_encode(uint32_t sequence, bool enabled, const double wrench[6],
                   uint8_t packet[GP_VFT_PACKET_SIZE]);

const char *gp_vft_fault_name(GpVirtualFtFault fault);
void gp_vft_reset(GpVirtualFtState *state);

/* Any malformed, out-of-order, invalid-range or non-finite packet latches
 * a fault and clears the wrench. No automatic recovery in this process. */
bool gp_vft_accept(GpVirtualFtState *state, const GpVirtualFtLimits *limits,
                   const uint8_t *data, size_t size, uint32_t received_us);

/* Read at the simulated Supervisor time (same monotonic clock as receipt).
 * Never extends freshness just because the control loop runs repeatedly.
 * A valid ENABLED zero packet is a normal release, not a disconnect. */
bool gp_vft_read(GpVirtualFtState *state, const GpVirtualFtLimits *limits,
                 uint32_t now_us, double wrench_base_tcp[6]);
#endif
