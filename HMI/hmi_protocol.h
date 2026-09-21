#ifndef HMI_PROTOCOL_H
#define HMI_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include <netinet/in.h>

#include "hmi_types.h"

typedef struct
{
    int commandSocket;
    int statusSocket;

    struct sockaddr_in controllerAddress;

    HmiControllerStatus status;
} HmiProtocol;

bool hmi_protocol_init(HmiProtocol *protocol);
void hmi_protocol_shutdown(HmiProtocol *protocol);

void hmi_protocol_poll_status(HmiProtocol *protocol);

bool hmi_protocol_controller_online(
    const HmiProtocol *protocol,
    double timeoutSeconds
);

bool hmi_protocol_send_plan(
    HmiProtocol *protocol,
    HmiPathType path,
    uint32_t sequence,
    const HmiPose *waypointA,
    const HmiPose *waypointB,
    const HmiPose *waypointC,
    float tcpSpeed,
    float tcpAccel,
    float tcpJerk
);

bool hmi_protocol_send_stop(
    HmiProtocol *protocol,
    uint32_t sequence
);

#endif /* HMI_PROTOCOL_H */
