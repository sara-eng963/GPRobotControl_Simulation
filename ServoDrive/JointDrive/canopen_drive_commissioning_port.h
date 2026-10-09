#ifndef CANOPEN_DRIVE_COMMISSIONING_PORT_H
#define CANOPEN_DRIVE_COMMISSIONING_PORT_H

#include "drive_commissioning_port.h"
#include "../../CANComm/CANopen/canopen_master.h"

DriveCommissioningPort canopen_drive_commissioning_port_make(CanopenMaster *master);

#endif /* CANOPEN_DRIVE_COMMISSIONING_PORT_H */
