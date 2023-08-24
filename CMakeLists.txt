cmake_minimum_required(VERSION 2.8.12)
project(pylon_camera_sync)

set(CATKIN_COMPONENTS
    actionlib
    camera_control_msgs
    camera_info_manager
    cv_bridge
    diagnostic_updater
    dnb_msgs
    image_geometry
    image_transport
    message_generation
    message_runtime
    pylon_camera
    roscpp
    roslaunch
    sensor_msgs
    std_msgs
    std_srvs)

# Look for the Pylon C++ API and fail if not there.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
if(NOT Pylon_FOUND)
  message(FATAL_ERROR "Could not find Pylon C++ API in ${PYLON_ROOT}")
else()
  message(STATUS "Found Pylon SDK at: ${PYLON_ROOT}")
  message(STATUS "Pylon_INCLUDE_DIRS: ${Pylon_INCLUDE_DIRS}")
  message(STATUS "Pylon_LIBRARIES: ${Pylon_LIBRARIES}")
endif()

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

add_service_files(FILES AckCam.srv)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ LIBRARIES
               ${PROJECT_NAME} CATKIN_DEPENDS ${CATKIN_COMPONENTS})

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Pylon_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/camera_gige.cpp 
  src/camera_manager.cpp 
  src/camera_node.cpp
  src/camera_parameter.cpp
  src/encoding_conversions.cpp 
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Pylon_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencpp)

add_executable(camera_sync_node src/main.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(camera_sync_node ${PROJECT_NAME})

install(
  TARGETS camera_sync_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/clang_tidy.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)
