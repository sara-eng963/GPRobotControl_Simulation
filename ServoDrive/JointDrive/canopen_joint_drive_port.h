#ifndef CANOPEN_JOINT_DRIVE_PORT_H
#define CANOPEN_JOINT_DRIVE_PORT_H

#include "joint_drive_port.h"
#include "../../CANComm/CANopen/canopen_master.h"

/*
 * Construct a non-owning adapter around the existing CANopen coordinator.
 * The master must outlive the port, and only the supervisor/drive owner may
 * call the port concurrently with other master operations.
 */
JointDrivePort canopen_joint_drive_port_make(CanopenMaster *master);

#endif /* CANOPEN_JOINT_DRIVE_PORT_H */
