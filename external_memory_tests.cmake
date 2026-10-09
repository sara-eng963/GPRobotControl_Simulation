# External-memory validation on the CAN/AVATAR branch.

add_executable(file_validated_storage_test
    Simulation/Tests/test_file_validated_storage.c
    Simulation/Storage/file_validated_storage.c
)

add_executable(validated_stream_buffer_test
    Simulation/Tests/test_validated_stream_buffer.c
    Simulation/Storage/validated_stream_buffer.c
)

add_executable(external_streaming_test
    Simulation/Tests/test_external_streaming.c
    Simulation/Storage/file_validated_storage.c
    Simulation/Storage/validated_stream_buffer.c
)
