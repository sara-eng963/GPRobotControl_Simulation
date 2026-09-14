#ifndef HMI_TYPES_H
#define HMI_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define HMI_NUM_POSE_VALUES 6
#define HMI_NUM_WAYPOINTS   3
#define HMI_NUM_JOINTS      6

typedef enum
{
    HMI_PATH_LINE = 0,
    HMI_PATH_ARC,
    HMI_PATH_FULL_CIRCLE
} HmiPathType;

typedef struct
{
    float value[HMI_NUM_POSE_VALUES];
} HmiPose;

typedef struct
{
    bool valid;
    double lastReceiveTime;

    uint32_t motionState;
    uint32_t lastSequence;
    uint32_t trajectoryIndex;
    uint32_t trajectoryCount;
    uint32_t wkc;
    uint32_t expectedWkc;
    uint16_t statusword[HMI_NUM_JOINTS];
} HmiControllerStatus;

#endif /* HMI_TYPES_H */
