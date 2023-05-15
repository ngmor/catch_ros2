function(catch_ros2_add_integration_test testname)
  cmake_parse_arguments(ARG
    "SKIP_TEST"
    "LAUNCH_FILE;TIMEOUT;RESULT_FILE;RESULT_FILE_PARAMETER_NAME"
    ""
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "catch_ros2_add_integration_test() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ${ament_cmake_FOUND})
    find_package(ament_cmake REQUIRED)
  endif()

  if(NOT ARG_LAUNCH_FILE)
    message(FATAL_ERROR
    "catch_ros2_add_integration_test() must be invoked with the LAUNCH_FILE argument")
  endif()
  if(NOT ARG_RESULT_FILE)
    set(ARG_RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml")
  endif()
  if(NOT ARG_TIMEOUT)
    set(ARG_TIMEOUT 60)
  endif()
  if(NOT ARG_TIMEOUT GREATER 0)
    message(FATAL_ERROR "catch_ros2_add_integration_test() the TIMEOUT argument must be a "
      "valid number and greater than zero")
  endif()
  if(NOT ARG_RESULT_FILE_PARAMETER_NAME)
    set(ARG_RESULT_FILE_PARAMETER_NAME "result_file")
  endif()

  ament_add_test(${testname}
    RESULT_FILE ${ARG_RESULT_FILE}
    COMMAND ros2 launch ${PROJECT_NAME} ${ARG_LAUNCH_FILE} ${ARG_RESULT_FILE_PARAMETER_NAME}:=${ARG_RESULT_FILE}
    RUNNER ${catch_ros2_DIR}/../scripts/run_test.py
  )
endfunction()