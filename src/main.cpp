#include <ros/console.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>

#include "camera_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pylon_camera_sync");
  ros::NodeHandle nh("~");
  CameraNode camera_node;

  return EXIT_SUCCESS;
}
