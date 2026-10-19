#include "../Guidance/virtual_ft_client.h"
#include "../Guidance/virtual_ft_controls.h"

#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

int main(void)
{
    const int receiver = socket(AF_INET, SOCK_DGRAM, 0);
    assert(receiver >= 0);
    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_port = htons(GP_VFT_PORT);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(receiver, (struct sockaddr *)&address, sizeof(address)) == 0);
    const struct timeval timeout = {.tv_sec=1, .tv_usec=0};
    assert(setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                      sizeof(timeout)) == 0);
    GpVirtualFtClient client;
    assert(gp_virtual_ft_client_open(&client));
    const GpVirtualFtLimits limits = {5.0, 1.0, GP_VFT_DEFAULT_TIMEOUT_US};
    GpVirtualFtState state={0};
    double wrench[6]={0};
    uint8_t packet[GP_VFT_PACKET_SIZE+1];
    for (unsigned i=0; i<3U; ++i) {
        wrench[0] = gp_virtual_ft_x_request(
            i==0, i==1, true, 0.5);
        assert(gp_virtual_ft_client_send(&client, wrench));
        const ssize_t count = recvfrom(receiver, packet, sizeof(packet),
                                       0, NULL, NULL);
        assert(count == GP_VFT_PACKET_SIZE);
        assert(gp_vft_accept(&state,&limits,packet,(size_t)count,1000+i*1000));
        double actual[6]={0};
        assert(gp_vft_read(&state,&limits,1001+i*1000,actual));
        assert(actual[0] == wrench[0]);
        for (unsigned j=1;j<6;++j) assert(actual[j] == 0.0);
    }
    gp_virtual_ft_client_close(&client);
    close(receiver);
    puts("PASS: same GUI sender UDP format and receiver for X+/X-/zero");
    return 0;
}
