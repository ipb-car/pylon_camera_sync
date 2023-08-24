# Actually the original File looks if PYLON_ROOT is set in the environment
# catkin_lint does not like this, so avoid this and simply set the default.
# Old (nice) code:
## set(PYLON_ROOT $ENV{PYLON_ROOT})
## if (NOT DEFINED ENV{PYLON_ROOT})
##    set(PYLON_ROOT "/opt/pylon")
## endif()
set(PYLON_ROOT "/opt/pylon")

set(_PYLON_CONFIG "${PYLON_ROOT}/bin/pylon-config")
if (EXISTS "${_PYLON_CONFIG}")
    set(Pylon_FOUND TRUE)
    execute_process(COMMAND ${_PYLON_CONFIG} --cflags-only-I OUTPUT_VARIABLE HEADERS_OUT)
    execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-l OUTPUT_VARIABLE LIBS_OUT)
    execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-L OUTPUT_VARIABLE LIBDIRS_OUT)
    string(REPLACE " " ";" HEADERS_OUT "${HEADERS_OUT}")
    string(REPLACE "-I" "" HEADERS_OUT "${HEADERS_OUT}")
    string(REPLACE "\n" "" Pylon_INCLUDE_DIRS "${HEADERS_OUT}")

    string(REPLACE " " ";" LIBS_OUT "${LIBS_OUT}")
    string(REPLACE "-l" "" LIBS_OUT "${LIBS_OUT}")
    string(REPLACE "\n" "" Pylon_LIBRARIES "${LIBS_OUT}")

    string(REPLACE " " ";" LIBDIRS_OUT "${LIBDIRS_OUT}")
    string(REPLACE "-L" "" LIBDIRS_OUT "${LIBDIRS_OUT}")
    string(REPLACE "\n" "" LIBDIRS_OUT "${LIBDIRS_OUT}")

    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    foreach (LIBDIR ${LIBDIRS_OUT})
        link_directories(${LIBDIR})
    endforeach()
else()
    set(Pylon_FOUND FALSE)
endif()