# Find ONNX Runtime
# TODO : remove hardcode
find_path(ONNXRUNTIME_INCLUDE_DIR 
  NAMES onnxruntime_cxx_api.h
  PATHS /home/alice/Downloads/onnxruntime-linux-x64-1.22.0/include/
)

find_library(ONNXRUNTIME_LIB 
  NAMES onnxruntime
  PATHS /home/alice/Downloads/onnxruntime-linux-x64-1.22.0/lib
)

set(controller_SRC
  RLController.cpp
  RLPolicyInterface.cpp
)

set(controller_HDR
  RLController.h
  RLPolicyInterface.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DRLCONTROLLER_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)

message(STATUS "DEBUG: Header path = ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "DEBUG: Lib path = ${ONNXRUNTIME_LIB}")
# Link ONNX Runtime if found
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  target_include_directories(${PROJECT_NAME} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${ONNXRUNTIME_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ONNX)
  message(STATUS "ONNX Runtime found, enabling ONNX support in RLController")
  message(STATUS "ONNX include: ${ONNXRUNTIME_INCLUDE_DIR}")
  message(STATUS "ONNX library: ${ONNXRUNTIME_LIB}")
else()
  message(WARNING "ONNX Runtime not found, only dummy policy will be available")
  message(STATUS "Install ONNX Runtime: sudo apt install libonnxruntime-dev")
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states) 
