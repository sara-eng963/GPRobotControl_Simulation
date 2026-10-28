/* Four-thread host test. This is not STM32 cache/MPU timing validation. */
#include "../ipc_mailbox.h"
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_MESSAGES 100000U
static IpcShared shared;
static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); ++failures; } } while(0)
static uint32_t checksum(uint32_t seq, unsigned direction)
{
    return (seq ^ UINT32_C(0xa4e5d7b1)) + direction;
}
typedef struct { unsigned direction; } ThreadArg;
static void *producer(void *arg)
{
    const unsigned direction = ((ThreadArg *)arg)->direction;
    for (uint32_t i = 0U; i < TEST_MESSAGES; ++i) {
        IpcMessage m = {0};
        m.type = direction ? IPC_CONTROLLER_STATUS : IPC_HMI_COMMAND;
        m.sequence = i + 1U;
        m.timestamp_ms = i * 2U;
        for (unsigned j = 0; j < IPC_WORDS; ++j)
            m.data[j] = checksum(i + j, direction);
        while (!(direction ? ipc_m7_send(&shared, &m) : ipc_m4_send(&shared, &m)))
            sched_yield();
    }
    return NULL;
}
static void *consumer(void *arg)
{
    const unsigned direction = ((ThreadArg *)arg)->direction;
    for (uint32_t i = 0U; i < TEST_MESSAGES; ++i) {
        IpcMessage m;
        while (!(direction ? ipc_m4_receive(&shared, &m) : ipc_m7_receive(&shared, &m)))
            sched_yield();
        if (m.sequence != i + 1U || m.timestamp_ms != i * 2U ||
            m.type != (direction ? IPC_CONTROLLER_STATUS : IPC_HMI_COMMAND)) {
            __atomic_add_fetch(&failures, 1, __ATOMIC_RELAXED);
            return NULL;
        }
        for (unsigned j = 0; j < IPC_WORDS; ++j)
            if (m.data[j] != checksum(i+j, direction)) {
                __atomic_add_fetch(&failures, 1, __ATOMIC_RELAXED);
                return NULL;
            }
    }
    return NULL;
}
int main(void)
{
    IpcMessage result = {0};
    IpcMessage input = { .type = IPC_HMI_COMMAND, .sequence = 1U };
    CHECK(!ipc_valid(&shared));
    CHECK(!ipc_m4_send(&shared, &input));
    ipc_initialize(&shared);
    CHECK(ipc_valid(&shared));
    CHECK(!ipc_m7_receive(&shared, &result));
    CHECK(!ipc_m4_receive(&shared, &result));
    CHECK(!ipc_m4_send(&shared, NULL));
    IpcMessage bad = { .type = 99U };
    CHECK(!ipc_m4_send(&shared, &bad));
    for (uint32_t i=0U; i<IPC_RING_CAPACITY; ++i) {
        input.sequence = i + 1U;
        CHECK(ipc_m4_send(&shared, &input));
    }
    CHECK(ipc_m4_to_m7_pending(&shared) == IPC_RING_CAPACITY);
    CHECK(!ipc_m4_send(&shared, &input));
    for (uint32_t i=0U; i<IPC_RING_CAPACITY; ++i) {
        CHECK(ipc_m7_receive(&shared, &result));
        CHECK(result.sequence == i + 1U);
    }
    CHECK(ipc_m4_to_m7_pending(&shared) == 0U);
    CHECK(!ipc_m7_receive(&shared, &result));
    ipc_initialize(&shared); /* quiescent only */
    ThreadArg args[2] = {{0U},{1U}};
    pthread_t threads[4];
    for (int i=0; i<2; ++i) CHECK(pthread_create(&threads[i], NULL, producer, &args[i]) == 0);
    for (int i=0; i<2; ++i) CHECK(pthread_create(&threads[i+2], NULL, consumer, &args[i]) == 0);
    for (int i=0; i<4; ++i) CHECK(pthread_join(threads[i], NULL) == 0);
    CHECK(ipc_m4_to_m7_pending(&shared) == 0U);
    CHECK(ipc_m7_to_m4_pending(&shared) == 0U);
    if (failures) return EXIT_FAILURE;
    puts("PASS: 100000 ordered messages per direction, concurrent M4/M7 ring test");
    return EXIT_SUCCESS;
}
