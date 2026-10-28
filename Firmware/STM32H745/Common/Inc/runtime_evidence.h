#ifndef RUNTIME_EVIDENCE_H
#define RUNTIME_EVIDENCE_H
/* P03 software-only authorization evidence. No hardware-stop guarantee.
 * All limits provisional pending producer/WCET measurements on STM32H745.
 * Only same-clock safety timestamps may be subtracted; CM4 timestamps are
 * NEVER compared against CM7 ticks. */
#include <stdbool.h>
#include <stdint.h>
#include "ipc_mailbox.h"
#include "../../CM7/Application/StateMachine/state_machine_types.h"

#define RUNTIME_SAFETY_MAX_AGE_MS UINT32_C(100)
#define RUNTIME_PEER_MAX_AGE_MS UINT32_C(50)
#define RUNTIME_CHALLENGE_MAX_RTT_MS UINT32_C(30)
#define RUNTIME_PEER_PROTOCOL_VERSION UINT32_C(2)

/* Commissioned verifier must authenticate sample identity, completed source
 * sequence and a timestamp in the EXACT CM7 monotonic-ms clock domain.
 * Unavailable verifier is explicitly non-authorizing. */
typedef struct {
    bool (*verify)(void *context, const RobotSafetySnapshot *sample,
                   uint32_t *producer_session, uint32_t *producer_sequence);
    void *context;
    uint32_t max_age_ms;
} RuntimeSafetyPort;
/* Must generate a reset-unique, nonzero CM7 session, across power loss.
 * No default issuer is installed. */
typedef struct {
    bool (*issue)(void *context, uint32_t *session);
    void *context;
} RuntimeIpcSessionPort;

/* An absent/failed issuer never creates a session. Prevents old SRAM4
 * messages from authorizing a new CM7 boot. */
static inline bool runtime_session_issue(const RuntimeIpcSessionPort *port,
                                          uint32_t *session)
{
    if (!session) return false;
    *session=0U;
    return port && port->issue && port->issue(port->context,session) &&
           *session!=0U;
}

typedef struct {
    uint32_t producer_session, last_sequence, last_source_ms, last_receipt_ms;
    bool accepted;
} RuntimeSafetyEvidence;

typedef struct {
    uint32_t expected_session;
    uint32_t last_sequence, last_response_ms;
    uint32_t challenge_counter, challenge_issued_ms, challenge_pending;
    uint32_t last_accepted_challenge;
    bool hello_seen, accepted, challenge_outstanding, compromised, have_sequence;
} RuntimePeerEvidence;

static inline bool runtime_forward(uint32_t next, uint32_t prior)
{
    const uint32_t delta = next - prior;
    return delta != 0U && delta < UINT32_C(0x80000000);
}
static inline bool runtime_age_valid(uint32_t now, uint32_t then,
                                     uint32_t limit)
{
    const uint32_t age = now - then;
    return limit != 0U && limit < UINT32_C(0x80000000) &&
           age < UINT32_C(0x80000000) && age <= limit;
}
static inline bool runtime_safety_accept(RuntimeSafetyEvidence *state,
    const RuntimeSafetyPort *port, const RobotSafetySnapshot *sample,
    uint32_t receipt_ms)
{
    uint32_t session=0U, sequence=0U;
    if (!state || !port || !port->verify || !sample ||
        !port->verify(port->context, sample, &session, &sequence) ||
        !session || (!state->accepted && !sequence) || !sample->statusValid ||
        (sample->motionPermitted && (sample->estopActive ||
              sample->protectiveStopActive || sample->globalFaultActive)) ||
        !runtime_age_valid(receipt_ms, sample->timestampMs, port->max_age_ms))
        return false;
    if (state->accepted &&
        (session != state->producer_session ||
         !runtime_forward(sequence, state->last_sequence) ||
         !runtime_forward(sample->timestampMs, state->last_source_ms)))
        return false;
    state->producer_session=session;
    state->last_sequence=sequence;
    state->last_source_ms=sample->timestampMs;
    state->last_receipt_ms=receipt_ms;
    state->accepted=true;
    return true;
}
static inline bool runtime_safety_fresh(const RuntimeSafetyEvidence *state,
    const RuntimeSafetyPort *port, uint32_t now_ms)
{
    return state && port && port->verify && state->accepted &&
           runtime_age_valid(now_ms,state->last_source_ms,port->max_age_ms) &&
           runtime_age_valid(now_ms,state->last_receipt_ms,port->max_age_ms);
}

/* CM4 HELLO: type IPC_CM4_HELLO, session=data[0], version=data[1],
 * data[2]=1 (CM4 service initialized). Sent once per CM4 application boot,
 * before progress or HMI. A second HELLO in a CM7 session permanently
 * compromises peer evidence; CM4 must not generate a reused boot marker. */
static inline bool runtime_peer_on_hello(RuntimePeerEvidence *state,
                                         const IpcMessage *msg)
{
    if (!state || !msg || !state->expected_session ||
        msg->type != IPC_CM4_HELLO || msg->data[0]!=state->expected_session ||
        msg->data[1]!=RUNTIME_PEER_PROTOCOL_VERSION || msg->data[2]!=1U ||
        msg->sequence!=1U)
        return false;
    if (state->hello_seen || state->compromised) {
        state->compromised=true;
        state->accepted=false;
        state->challenge_outstanding=false;
        return false;
    }
    state->hello_seen=true;
    return true;
}

/* One CM7-local outstanding challenge at a time. Repeated status frames
 * retransmit that same challenge; no new work is inferred from an echo.
 * On timeout a fresh challenge can be issued for STARTUP diagnostics, but
 * active authorization must already be revoked by runtime_peer_fresh().
 * Counter never wraps within a CM7 session (fail closed at UINT32_MAX). */
static inline bool runtime_peer_challenge(RuntimePeerEvidence *state,
                                         uint32_t now_ms, uint32_t *out)
{
    if (!state || !out || !state->expected_session || !state->hello_seen ||
        state->compromised) return false;
    if (state->challenge_outstanding &&
        !runtime_age_valid(now_ms,state->challenge_issued_ms,
                           RUNTIME_CHALLENGE_MAX_RTT_MS)) {
        state->challenge_outstanding=false;
        state->accepted=false;
    }
    if (!state->challenge_outstanding) {
        if (state->challenge_counter==UINT32_MAX) {
            state->compromised=true;
            state->accepted=false;
            return false;
        }
        state->challenge_pending=++state->challenge_counter;
        state->challenge_issued_ms=now_ms;
        state->challenge_outstanding=true;
    }
    *out=state->challenge_pending;
    return true;
}

/* CM4 completion response: data[0] CM7 session, [1] protocol version,
 * [2]=SUP_PROGRESS_SERVICE, [3]=completed_ok, [4]=valid, [5]=response_ok,
 * [6] exact outstanding CM7-issued challenge. sequence is a strictly
 * advancing CM4 completed-service sequence. CM4 timestamp is diagnostic.
 * Receipt alone never renews freshness without a timely local challenge. */
static inline bool runtime_peer_accept(RuntimePeerEvidence *state,
    const IpcMessage *msg, uint32_t now_ms)
{
    if (!state || !msg || !state->expected_session || !state->hello_seen ||
        state->compromised || !state->challenge_outstanding ||
        !runtime_age_valid(now_ms,state->challenge_issued_ms,
                           RUNTIME_CHALLENGE_MAX_RTT_MS) ||
        msg->type!=IPC_CM4_PROGRESS ||
        msg->data[0]!=state->expected_session ||
        msg->data[1]!=RUNTIME_PEER_PROTOCOL_VERSION ||
        msg->data[2]!=3U || msg->data[3]!=1U ||
        msg->data[4]!=1U || msg->data[5]!=1U ||
        msg->data[6]!=state->challenge_pending)
        return false;
    /* Preserve ordering even after an expired response and STARTUP retry. */
    if (state->have_sequence &&
        !runtime_forward(msg->sequence,state->last_sequence)) return false;
    state->last_sequence=msg->sequence;
    state->have_sequence=true;
    state->last_response_ms=now_ms;
    state->last_accepted_challenge=state->challenge_pending;
    state->challenge_outstanding=false;
    state->accepted=true;
    return true;
}
static inline bool runtime_peer_fresh(const RuntimePeerEvidence *state,
                                      uint32_t now_ms)
{
    return state && state->expected_session && state->hello_seen &&
        !state->compromised && state->accepted &&
        runtime_age_valid(now_ms,state->last_response_ms,
                          RUNTIME_PEER_MAX_AGE_MS) &&
        (!state->challenge_outstanding ||
         runtime_age_valid(now_ms,state->challenge_issued_ms,
                           RUNTIME_CHALLENGE_MAX_RTT_MS));
}
#endif /* RUNTIME_EVIDENCE_H */
