/*
 * Path Validation integration/capacity characterization entry point.
 *
 * Test 7.6 lives with the other simulator/external-memory tests, while the
 * root CMakeLists.txt already exposes a conditional `path_validation_test`
 * target at this path. Include the Test 7.6 translation unit here so the
 * existing target can build the real Path Validation + ControlCore pipeline
 * without duplicating the test implementation.
 */
#include "../../Simulation/Tests/test_trajectory_storage_capacity.c"
