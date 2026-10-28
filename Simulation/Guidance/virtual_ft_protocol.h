#ifndef VIRTUAL_FT_PROTOCOL_H
#define VIRTUAL_FT_PROTOCOL_H

/* Host simulation ONLY. Atomic datagram: 10 big-endian 32-bit words.
 * [magic,version,sequence,enable,Fx,Fy,Fz,Tx,Ty,Tz].
 * The last six words are IEEE-754 float32 bit patterns, base-frame wrench
 * ABOUT THE TCP, not joint or Cartesian position commands. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIRTUAL_FT_PORT 5013U
#define VIRTUAL_FT_MAGIC UINT32_C(0x56465431) /* VFT1 */
#define VIRTUAL_FT_VERSION 1U
#define VIRTUAL_FT_PACKET_SIZE 40U
#define VIRTUAL_FT_DEFAULT_TIMEOUT_US 150000U

typedef struct {
    double max_force_N;
    double max_torque_Nm;
    uint32_t timeout_us;
} VirtualFtLimits;

typedef enum {
    VIRTUAL_FT_FAULT_NONE = 0,
    VIRTUAL_FT_FAULT_INVALID_PACKET,
    VIRTUAL_FT_FAULT_SEQUENCE,
    VIRTUAL_FT_FAULT_TIMEOUT
} VirtualFtFault;

typedef struct {
    double wrench_base_tcp[6];
    uint32_t sequence;
    uint32_t received_us;
    bool connected;
    bool enabled;
    bool fault_latched;
    VirtualFtFault fault_reason;
} VirtualFtState;

/* Serialize the same packet format the existing receiver validates.
 * Does not impose sensor limits: receiver owns the authoritative limits. */
bool virtual_ft_encode(uint32_t sequence, bool enabled, const double wrench[6],
                   uint8_t packet[VIRTUAL_FT_PACKET_SIZE]);

const char *virtual_ft_fault_name(VirtualFtFault fault);
void virtual_ft_reset(VirtualFtState *state);

/* Any malformed, out-of-order, invalid-range or non-finite packet latches
 * a fault and clears the wrench. No automatic recovery in this process. */
bool virtual_ft_accept(VirtualFtState *state, const VirtualFtLimits *limits,
                   const uint8_t *data, size_t size, uint32_t received_us);

/* Read at the simulated Supervisor time (same monotonic clock as receipt).
 * Never extends freshness just because the control loop runs repeatedly.
 * A valid ENABLED zero packet is a normal release, not a disconnect. */
bool virtual_ft_read(VirtualFtState *state, const VirtualFtLimits *limits,
                 uint32_t now_us, double wrench_base_tcp[6]);
#endif
