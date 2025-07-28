add_fsm_state_simple(RLController_Initial)
add_fsm_state_simple(RLExecution)

set(FSM_STATES RLController_Initial RLExecution)

foreach(state ${FSM_STATES})
  message(STATUS "Configuring FSM target: ${state}")
  target_include_directories(${state} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(${state} PRIVATE ${ONNXRUNTIME_LIB})
endforeach()
