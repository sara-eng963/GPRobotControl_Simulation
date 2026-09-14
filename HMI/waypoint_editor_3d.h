#ifndef WAYPOINT_EDITOR_3D_H
#define WAYPOINT_EDITOR_3D_H

#include <stdbool.h>

#include "raylib.h"

#include "hmi_theme.h"
#include "hmi_types.h"

typedef enum
{
    HMI_TEACH_VIEW_PERSPECTIVE = 0,
    HMI_TEACH_VIEW_TOP,
    HMI_TEACH_VIEW_FRONT,
    HMI_TEACH_VIEW_SIDE
} HmiTeachView;

typedef enum
{
    HMI_EDITOR_RESULT_NONE = 0,
    HMI_EDITOR_RESULT_APPLIED,
    HMI_EDITOR_RESULT_CANCELLED
} HmiWaypointEditorResult;

typedef struct
{
    bool open;
    HmiPathType path;
    HmiPose working[HMI_NUM_WAYPOINTS];

    int activeWaypoint;
    HmiTeachView view;

    Camera3D camera;
    RenderTexture2D renderTarget;
    bool renderTargetReady;

    Vector3 orbitTarget;
    float orbitYaw;
    float orbitPitch;
    float orbitDistance;

    bool orbitDragging;
    Vector2 previousMouse;

    int dragAxis;
    Vector3 dragOriginRender;
    float dragStartParameter;

    HmiNumericField xyzFields[3];
    bool xyzFieldsDirty;

    char message[160];
} HmiWaypointEditor3D;

bool hmi_waypoint_editor_init(HmiWaypointEditor3D *editor);
void hmi_waypoint_editor_shutdown(HmiWaypointEditor3D *editor);

void hmi_waypoint_editor_open(
    HmiWaypointEditor3D *editor,
    HmiPathType path,
    const HmiPose waypoints[HMI_NUM_WAYPOINTS]
);

bool hmi_waypoint_editor_is_open(
    const HmiWaypointEditor3D *editor
);

HmiWaypointEditorResult hmi_waypoint_editor_frame(
    HmiWaypointEditor3D *editor,
    Rectangle bounds,
    Vector2 mouse,
    HmiPose outputWaypoints[HMI_NUM_WAYPOINTS]
);

#endif /* WAYPOINT_EDITOR_3D_H */
