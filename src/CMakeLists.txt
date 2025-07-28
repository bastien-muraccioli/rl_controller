# ---- Check for ONNX Runtime ----

set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")

if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT $ENV{ONNXRUNTIME_ROOT})
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}
  PATH_SUFFIXES include
  PATHS
    /usr/include
    /usr/local/include
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}
  PATH_SUFFIXES lib
  PATHS
    /usr/lib
    /usr/local/lib
)

if(NOT ONNXRUNTIME_INCLUDE_DIR)
  message(FATAL_ERROR "ONNX Runtime headers not found. Set -DONNXRUNTIME_ROOT or install it.")
endif()

if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime library not found. Set -DONNXRUNTIME_ROOT or install it.")
endif()

message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime lib: ${ONNXRUNTIME_LIB}")

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

# ---- Link ONNX to both targets ----

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