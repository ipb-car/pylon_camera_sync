#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <GenApi/GenApi.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>
#include <pylon/gige/GigETransportLayer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

#include <sstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include "camera_gige.hpp"
#include "camera_parameter.hpp"

class CameraManager {
 public:
  CameraManager();
  ~CameraManager();

  bool init();
  uint64_t getCurrTimeStamp();
  bool enableFreeRunMode();
  bool startGrabbing();
  bool stopGrabbing();
  bool spin();

  void setupInitialCameraInfo(sensor_msgs::CameraInfo &cam_info_msg);
  void setupRectification();
  std::string getDeviceUserId();

 private:
  ros::NodeHandle m_nh;
  PylonCameraParameter m_camera_parameter_set;
  std::shared_ptr<image_transport::ImageTransport> m_it;
  image_transport::CameraPublisher m_img_raw_pub;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager;
  sensor_msgs::Image m_img_raw_msg;

  std::shared_ptr<CameraGigE> m_camera_gige;
};

#endif  // CAMERAMANAGER_H