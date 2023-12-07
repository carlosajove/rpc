
add_executable(TEST0 test_mpc_dyn.cpp)
target_link_libraries(TEST0 rpc-alip-planner rpc-util)

message(${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(TEST0 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
)