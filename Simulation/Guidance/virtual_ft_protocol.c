#include "virtual_ft_protocol.h"

#include <math.h>
#include <string.h>

static uint32_t read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void write_be32(uint8_t *p, uint32_t word)
{
    p[0] = (uint8_t)(word >> 24);
    p[1] = (uint8_t)(word >> 16);
    p[2] = (uint8_t)(word >> 8);
    p[3] = (uint8_t)word;
}

bool virtual_ft_encode(uint32_t sequence, bool enabled, const double wrench[6],
                   uint8_t packet[VIRTUAL_FT_PACKET_SIZE])
{
    if (packet == NULL || wrench == NULL) return false;
    float values[6];
    for (size_t i = 0U; i < 6U; ++i) {
        if (!isfinite(wrench[i]) || (!enabled && wrench[i] != 0.0))
            return false;
        values[i] = (float)wrench[i];
        if (!isfinite(values[i])) return false;
    }
    write_be32(packet, VIRTUAL_FT_MAGIC);
    write_be32(packet + 4U, VIRTUAL_FT_VERSION);
    write_be32(packet + 8U, sequence);
    write_be32(packet + 12U, enabled ? 1U : 0U);
    for (size_t i = 0U; i < 6U; ++i) {
        uint32_t bits = 0U;
        memcpy(&bits, &values[i], sizeof(bits));
        write_be32(packet + 16U + 4U*i, bits);
    }
    return true;
}

static bool limits_valid(const VirtualFtLimits *l)
{
    return l != NULL && isfinite(l->max_force_N) &&
           isfinite(l->max_torque_Nm) && l->max_force_N > 0.0 &&
           l->max_torque_Nm > 0.0 && l->timeout_us > 0U &&
           l->timeout_us < UINT32_C(0x80000000);
}

const char *virtual_ft_fault_name(VirtualFtFault reason)
{
    switch (reason) {
    case VIRTUAL_FT_FAULT_NONE: return "none";
    case VIRTUAL_FT_FAULT_INVALID_PACKET: return "invalid_packet_or_range";
    case VIRTUAL_FT_FAULT_SEQUENCE: return "duplicate_or_out_of_order_sequence";
    case VIRTUAL_FT_FAULT_TIMEOUT: return "wrench_input_timeout";
    default: return "unknown";
    }
}

static bool trip(VirtualFtState *state, VirtualFtFault reason)
{
    if (state != NULL) {
        state->fault_latched = true;
        state->fault_reason = reason;
        state->enabled = false;
        memset(state->wrench_base_tcp, 0, sizeof(state->wrench_base_tcp));
    }
    return false;
}

void virtual_ft_reset(VirtualFtState *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

bool virtual_ft_accept(VirtualFtState *state, const VirtualFtLimits *limits,
                   const uint8_t *data, size_t size, uint32_t received_us)
{
    if (state == NULL || state->fault_latched) return false;
    if (!limits_valid(limits) || data == NULL || size != VIRTUAL_FT_PACKET_SIZE)
        return trip(state, VIRTUAL_FT_FAULT_INVALID_PACKET);
    const uint32_t magic = read_be32(data);
    const uint32_t version = read_be32(data + 4U);
    const uint32_t sequence = read_be32(data + 8U);
    const uint32_t enabled = read_be32(data + 12U);
    if (magic != VIRTUAL_FT_MAGIC || version != VIRTUAL_FT_VERSION ||
        enabled > 1U)
        return trip(state, VIRTUAL_FT_FAULT_INVALID_PACKET);
    if (state->connected && (int32_t)(sequence - state->sequence) <= 0)
        return trip(state, VIRTUAL_FT_FAULT_SEQUENCE);

    double candidate[6];
    for (size_t i = 0U; i < 6U; ++i) {
        const uint32_t bits = read_be32(data + 16U + i * 4U);
        float f;
        memcpy(&f, &bits, sizeof(f));
        candidate[i] = (double)f;
        const double limit = i < 3U ? limits->max_force_N :
                                       limits->max_torque_Nm;
        if (!isfinite(candidate[i]) || fabs(candidate[i]) > limit ||
            (enabled == 0U && candidate[i] != 0.0))
            return trip(state, VIRTUAL_FT_FAULT_INVALID_PACKET);
    }
    memcpy(state->wrench_base_tcp, candidate, sizeof(candidate));
    state->sequence = sequence;
    state->received_us = received_us;
    state->connected = true;
    state->enabled = enabled == 1U;
    return true;
}

bool virtual_ft_read(VirtualFtState *state, const VirtualFtLimits *limits,
                 uint32_t now_us, double wrench_base_tcp[6])
{
    if (wrench_base_tcp != NULL)
        memset(wrench_base_tcp, 0, 6U * sizeof(double));
    if (state == NULL || wrench_base_tcp == NULL ||
        !limits_valid(limits) || state->fault_latched)
        return false;
    if (!state->connected || !state->enabled) return false;
    const uint32_t age = now_us - state->received_us;
    if (age >= UINT32_C(0x80000000) || age > limits->timeout_us)
        return trip(state, VIRTUAL_FT_FAULT_TIMEOUT);
    memcpy(wrench_base_tcp, state->wrench_base_tcp, 6U * sizeof(double));
    return true;
}
