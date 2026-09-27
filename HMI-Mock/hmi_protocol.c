#include "hmi_protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define CONTROLLER_IP   "127.0.0.1"
#define CONTROLLER_PORT 5006
#define STATUS_PORT     5007

#define PACKET_MAGIC        0x52425432U /* RBT2 */
#define STATUS_PACKET_MAGIC 0x53544132U /* STA2 */
#define STATUS_WORD_COUNT   13U

#define LINE_PLAN_WORD_COUNT     18U
#define CIRCULAR_PLAN_WORD_COUNT 24U
#define STOP_WORD_COUNT           3U

typedef enum
{
    COMMAND_PLAN_AND_RUN_LINE        = 1,
    COMMAND_STOP                     = 2,
    COMMAND_PLAN_AND_RUN_ARC         = 3,
    COMMAND_PLAN_AND_RUN_FULL_CIRCLE = 4
} CommandType;

static double monotonic_seconds(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0.0;
    }

    return
        (double)ts.tv_sec +
        (double)ts.tv_nsec / 1000000000.0;
}

static uint32_t float_to_network_word(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return htonl(bits);
}

bool hmi_protocol_init(HmiProtocol *protocol)
{
    if (protocol == NULL)
    {
        return false;
    }

    memset(protocol, 0, sizeof(*protocol));
    protocol->commandSocket = -1;
    protocol->statusSocket = -1;

    protocol->commandSocket = socket(AF_INET, SOCK_DGRAM, 0);

    if (protocol->commandSocket < 0)
    {
        perror("HMI command socket");
        return false;
    }

    memset(
        &protocol->controllerAddress,
        0,
        sizeof(protocol->controllerAddress)
    );

    protocol->controllerAddress.sin_family = AF_INET;
    protocol->controllerAddress.sin_port = htons(CONTROLLER_PORT);

    if (
        inet_pton(
            AF_INET,
            CONTROLLER_IP,
            &protocol->controllerAddress.sin_addr
        ) != 1
    )
    {
        fprintf(stderr, "Invalid controller IP\n");
        hmi_protocol_shutdown(protocol);
        return false;
    }

    protocol->statusSocket = socket(AF_INET, SOCK_DGRAM, 0);

    if (protocol->statusSocket < 0)
    {
        perror("HMI status socket");
        hmi_protocol_shutdown(protocol);
        return false;
    }

    struct sockaddr_in statusAddress;
    memset(&statusAddress, 0, sizeof(statusAddress));

    statusAddress.sin_family = AF_INET;
    statusAddress.sin_port = htons(STATUS_PORT);
    statusAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    if (
        bind(
            protocol->statusSocket,
            (struct sockaddr *)&statusAddress,
            sizeof(statusAddress)
        ) < 0
    )
    {
        perror("HMI status bind");
        hmi_protocol_shutdown(protocol);
        return false;
    }

    return true;
}

void hmi_protocol_shutdown(HmiProtocol *protocol)
{
    if (protocol == NULL)
    {
        return;
    }

    if (protocol->statusSocket >= 0)
    {
        close(protocol->statusSocket);
        protocol->statusSocket = -1;
    }

    if (protocol->commandSocket >= 0)
    {
        close(protocol->commandSocket);
        protocol->commandSocket = -1;
    }
}

void hmi_protocol_poll_status(HmiProtocol *protocol)
{
    if (protocol == NULL || protocol->statusSocket < 0)
    {
        return;
    }

    for (;;)
    {
        uint32_t packet[STATUS_WORD_COUNT];

        ssize_t received =
            recvfrom(
                protocol->statusSocket,
                packet,
                sizeof(packet),
                MSG_DONTWAIT,
                NULL,
                NULL
            );

        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }

            perror("HMI status recvfrom");
            break;
        }

        if (received != (ssize_t)sizeof(packet))
        {
            continue;
        }

        if (ntohl(packet[0]) != STATUS_PACKET_MAGIC)
        {
            continue;
        }

        protocol->status.motionState = ntohl(packet[1]);
        protocol->status.lastSequence = ntohl(packet[2]);
        protocol->status.trajectoryIndex = ntohl(packet[3]);
        protocol->status.trajectoryCount = ntohl(packet[4]);
        protocol->status.wkc = ntohl(packet[5]);
        protocol->status.expectedWkc = ntohl(packet[6]);

        for (int joint = 0; joint < HMI_NUM_JOINTS; joint++)
        {
            protocol->status.statusword[joint] =
                (uint16_t)ntohl(packet[7 + joint]);
        }

        protocol->status.valid = true;
        protocol->status.lastReceiveTime = monotonic_seconds();
    }
}

bool hmi_protocol_controller_online(
    const HmiProtocol *protocol,
    double timeoutSeconds
)
{
    if (
        protocol == NULL ||
        !protocol->status.valid ||
        timeoutSeconds <= 0.0
    )
    {
        return false;
    }

    return
        monotonic_seconds() -
        protocol->status.lastReceiveTime <
        timeoutSeconds;
}

static uint32_t command_for_path(HmiPathType path)
{
    switch (path)
    {
        case HMI_PATH_LINE:
            return COMMAND_PLAN_AND_RUN_LINE;

        case HMI_PATH_ARC:
            return COMMAND_PLAN_AND_RUN_ARC;

        case HMI_PATH_FULL_CIRCLE:
            return COMMAND_PLAN_AND_RUN_FULL_CIRCLE;

        default:
            return 0U;
    }
}

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
)
{
    if (
        protocol == NULL ||
        protocol->commandSocket < 0 ||
        waypointA == NULL ||
        waypointB == NULL
    )
    {
        return false;
    }

    uint32_t command = command_for_path(path);

    if (command == 0U)
    {
        return false;
    }

    uint32_t packet[CIRCULAR_PLAN_WORD_COUNT];
    memset(packet, 0, sizeof(packet));

    packet[0] = htonl(PACKET_MAGIC);
    packet[1] = htonl(command);
    packet[2] = htonl(sequence);

    int index = 3;

    for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
    {
        packet[index++] = float_to_network_word(waypointA->value[i]);
    }

    for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
    {
        packet[index++] = float_to_network_word(waypointB->value[i]);
    }

    if (path != HMI_PATH_LINE)
    {
        if (waypointC == NULL)
        {
            return false;
        }

        for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
        {
            packet[index++] = float_to_network_word(waypointC->value[i]);
        }
    }

    packet[index++] = float_to_network_word(tcpSpeed);
    packet[index++] = float_to_network_word(tcpAccel);
    packet[index++] = float_to_network_word(tcpJerk);

    size_t wordCount =
        path == HMI_PATH_LINE
            ? LINE_PLAN_WORD_COUNT
            : CIRCULAR_PLAN_WORD_COUNT;

    ssize_t sent =
        sendto(
            protocol->commandSocket,
            packet,
            wordCount * sizeof(uint32_t),
            0,
            (const struct sockaddr *)&protocol->controllerAddress,
            sizeof(protocol->controllerAddress)
        );

    return
        sent ==
        (ssize_t)(wordCount * sizeof(uint32_t));
}

bool hmi_protocol_send_stop(
    HmiProtocol *protocol,
    uint32_t sequence
)
{
    if (protocol == NULL || protocol->commandSocket < 0)
    {
        return false;
    }

    uint32_t packet[STOP_WORD_COUNT];

    packet[0] = htonl(PACKET_MAGIC);
    packet[1] = htonl((uint32_t)COMMAND_STOP);
    packet[2] = htonl(sequence);

    ssize_t sent =
        sendto(
            protocol->commandSocket,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)&protocol->controllerAddress,
            sizeof(protocol->controllerAddress)
        );

    return sent == (ssize_t)sizeof(packet);
}
