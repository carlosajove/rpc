add_library(rpc-lmpc-handler lmpc_handler.cpp lmpc_handler.hpp)
target_link_libraries(rpc-lmpc-handler PUBLIC rpc-dcm-planner
    PUBLIC rpc-pin-robot-system
    PUBLIC rpc-util
    PUBLIC rpc-alip-planner)

install(TARGETS rpc-lmpc-handler DESTINATION "${INSTALL_LIB_DIR}")
install(FILES lmpc_handler.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/model_predictive_controller/lmpc")


