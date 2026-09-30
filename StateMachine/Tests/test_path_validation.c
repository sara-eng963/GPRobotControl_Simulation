/*
 * Path Validation integration/capacity characterization entry point.
 *
 * Test 7.6 lives with the other simulator/external-memory tests, while the
 * root CMakeLists.txt already exposes a conditional `path_validation_test`
 * target at this path. Include the ControlCore declarations used directly by
 * the capacity test before including its translation unit.
 */
#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/math3d.h"
#include "../../Simulation/Tests/test_trajectory_storage_capacity.c"
