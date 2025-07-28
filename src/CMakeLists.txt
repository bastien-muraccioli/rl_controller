# Find ONNX Runtime
# TODO : remove hardcode

find_path(ONNXRUNTIME_INCLUDE_DIR 
  NAMES onnxruntime_cxx_api.h
  PATHS /home/alice/Downloads/onnxruntime-linux-x64-1.22.0/include
)

find_library(ONNXRUNTIME_LIB 
  NAMES onnxruntime
  PATHS /home/alice/Downloads/onnxruntime-linux-x64-1.22.0/lib
)

if(NOT ONNXRUNTIME_INCLUDE_DIR)
  message(FATAL_ERROR "ONNX Runtime headers not found")
endif()

if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime library not found")
endif()

message(STATUS "ONNX Runtime found, enabling ONNX support in RLController")
message(STATUS "ONNX include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX library: ${ONNXRUNTIME_LIB}")

# ---- Sources ----

set(controller_SRC
  RLController.cpp
  RLPolicyInterface.cpp
)

set(controller_HDR
  RLController.h
  RLPolicyInterface.h
)

# ---- Main controller library ----

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DRLCONTROLLER_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)

# ---- Plugin target (mc_rtc controller entry point) ----

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

# ---- Apply ONNX includes/libs to both targets ----

foreach(tgt IN ITEMS ${PROJECT_NAME} ${PROJECT_NAME}_controller)
  target_include_directories(${tgt} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(${tgt} PRIVATE ${ONNXRUNTIME_LIB})
endforeach()

# ---- Install ----

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin
)

# ---- Controller state machine ----

add_subdirectory(states)