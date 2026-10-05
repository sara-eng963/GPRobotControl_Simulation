#include "../can_backend.h"

#include <stdio.h>
#include <string.h>


typedef struct
{
    CanFrame stored_frame;
    bool frame_available;

} FakeCanContext;


static CanBackendResult fake_send(
    void *context,
    const CanFrame *frame
)
{
    FakeCanContext *fake =
        (FakeCanContext *)context;

    fake->stored_frame = *frame;
    fake->frame_available = true;

    return CAN_BACKEND_OK;
}


static CanBackendResult fake_receive(
    void *context,
    CanFrame *frame
)
{
    FakeCanContext *fake =
        (FakeCanContext *)context;

    if (!fake->frame_available)
    {
        return CAN_BACKEND_WOULD_BLOCK;
    }

    *frame = fake->stored_frame;

    fake->frame_available = false;

    return CAN_BACKEND_OK;
}


static void fake_close(
    void *context
)
{
    (void)context;
}


int main(void)
{
    FakeCanContext context = {0};

    CanBackend backend =
    {
        .context = &context,
        .send = fake_send,
        .receive = fake_receive,
        .close = fake_close
    };


    CanFrame tx =
    {
        .id = 0x501,
        .dlc = 4,
        .data = {
            0x50,
            0xC3,
            0x00,
            0x00
        }
    };


    CanFrame rx = {0};


    if (!can_backend_valid(&backend))
    {
        printf("[FAIL] Backend validation\n");
        return 1;
    }

    printf("[PASS] Backend validation\n");


    if (
        can_backend_send(
            &backend,
            &tx
        ) != CAN_BACKEND_OK
    )
    {
        printf("[FAIL] CAN send\n");
        return 1;
    }

    printf("[PASS] CAN send\n");


    if (
        can_backend_receive(
            &backend,
            &rx
        ) != CAN_BACKEND_OK
    )
    {
        printf("[FAIL] CAN receive\n");
        return 1;
    }

    printf("[PASS] CAN receive\n");


    if (
        rx.id != tx.id ||
        rx.dlc != tx.dlc ||
        memcmp(
            rx.data,
            tx.data,
            tx.dlc
        ) != 0
    )
    {
        printf("[FAIL] Frame preserved\n");
        return 1;
    }

    printf("[PASS] Frame preserved\n");


    if (
        can_backend_receive(
            &backend,
            &rx
        ) != CAN_BACKEND_WOULD_BLOCK
    )
    {
        printf("[FAIL] Empty receive behavior\n");
        return 1;
    }

    printf("[PASS] Empty receive behavior\n");

    can_backend_close(&backend);

    printf("\nTests failed: 0\n");

    return 0;
}