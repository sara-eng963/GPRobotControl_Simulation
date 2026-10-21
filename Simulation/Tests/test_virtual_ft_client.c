#include "../Guidance/virtual_ft_client.h"
#include "../Guidance/virtual_ft_controls.h"

#include <arpa/inet.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static void send_and_check(GpVirtualFtClient *client, int receiver,
                           GpVirtualFtState *state,
                           const GpVirtualFtLimits *limits,
                           const double expected[6], uint32_t timestamp)
{
    uint8_t packet[GP_VFT_PACKET_SIZE + 1U];
    assert(gp_virtual_ft_client_send(client, expected));
    const ssize_t count = recvfrom(receiver, packet, sizeof(packet), 0, NULL, NULL);
    assert(count == GP_VFT_PACKET_SIZE);
    assert(gp_vft_accept(state, limits, packet, (size_t)count, timestamp));
    double actual[6] = {0};
    assert(gp_vft_read(state, limits, timestamp + 1U, actual));
    for (unsigned i = 0U; i < 6U; ++i)
        assert(fabs(actual[i] - expected[i]) < 1e-6);
}

int main(void)
{
    const int receiver = socket(AF_INET, SOCK_DGRAM, 0);
    assert(receiver >= 0);
    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_port = htons(GP_VFT_PORT);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(receiver, (struct sockaddr *)&address, sizeof(address)) == 0);
    const struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    assert(setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                      sizeof(timeout)) == 0);
    GpVirtualFtClient client;
    assert(gp_virtual_ft_client_open(&client));
    const GpVirtualFtLimits limits = {5.0, 1.0, GP_VFT_DEFAULT_TIMEOUT_US};
    GpVirtualFtState state = {0};
    GpVirtualFtKeys keys = {0};
    double wrench[6];
    uint32_t timestamp = 1000U;
    for (unsigned axis = 0U; axis < 6U; ++axis) {
        keys.positive[axis] = true;
        gp_virtual_ft_wrench_request(&keys, true, 0.5, 0.25, wrench);
        send_and_check(&client, receiver, &state, &limits, wrench, timestamp++);
        keys.positive[axis] = false;
        keys.negative[axis] = true;
        gp_virtual_ft_wrench_request(&keys, true, 0.5, 0.25, wrench);
        send_and_check(&client, receiver, &state, &limits, wrench, timestamp++);
        keys.negative[axis] = false;
    }
    /* Combined translation and rotation, then clear one opposing pair. */
    keys.positive[0] = true;
    keys.negative[1] = true;
    keys.positive[3] = true;
    keys.negative[5] = true;
    gp_virtual_ft_wrench_request(&keys, true, 1.5, 0.5, wrench);
    send_and_check(&client, receiver, &state, &limits, wrench, timestamp++);
    keys.negative[0] = true;
    gp_virtual_ft_wrench_request(&keys, true, 1.5, 0.5, wrench);
    assert(wrench[0] == 0.0 && wrench[1] == -1.5 && wrench[3] == 0.5);
    send_and_check(&client, receiver, &state, &limits, wrench, timestamp++);
    gp_virtual_ft_wrench_request(&keys, false, 1.5, 0.5, wrench);
    send_and_check(&client, receiver, &state, &limits, wrench, timestamp++);
    gp_virtual_ft_client_close(&client);
    close(receiver);
    puts("PASS: actual GUI UDP sender -> six-axis protocol receiver, combos and zero");
    return 0;
}
