add_subdirectory("src/codegen_alip_mpc")

# Find and use CasADi   COULD PUT THIS ON THE FIRST CMAKELIST
find_package(casadi REQUIRED)


#file(GLOB sources "src/*.cpp")
file(GLOB headers "include/*.hpp")



add_library(rpc-dcm-alip-planner ${CMAKE_CURRENT_SOURCE_DIR}/src/NewStep_mpc.cpp)
target_include_directories(rpc-dcm-alip-planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rpc-dcm-alip-planner PUBLIC casadi)



install(TARGETS rpc-dcm-alip-planner DESTINATION ${INSTALL_LIB_DIR})
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/planner/locomotion/alip_mpc")


