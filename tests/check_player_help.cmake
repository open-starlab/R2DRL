# --help returns before team strategy loading, shared memory or server connect.
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "LD_LIBRARY_PATH=${PLAYER_LIBS}" "${PLAYER}" --help
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 10)
set(help "${output}\n${errors}")
# Upstream main_player returns EXIT_FAILURE when help makes init() return false.
if(NOT result EQUAL 1 OR NOT help MATCHES "print help message")
    message(FATAL_ERROR "Player could not print help with bundled libraries: ${result}\n${help}")
endif()
message(STATUS "Player loads with bundled runtime libraries and prints help; no match started")
