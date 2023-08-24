#ifndef CAMERAPROCESS_H
#define CAMERAPROCESS_H

#include <GenApi/GenApi.h>
#include <pylon/PylonIncludes.h>
#include <pylon_camera_sync/AckCam.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "camera_manager.hpp"

class CameraNode {
 public:
  CameraNode();
  ~CameraNode();
  void run();

 private:
  bool defineSovereignty();
  bool syncCameras();

  // handlers
  bool handleMasterExistCamera(pylon_camera_sync::AckCam::Request &req,
                               pylon_camera_sync::AckCam::Response &res);
  bool handleMasterSyncCamera(pylon_camera_sync::AckCam::Request &req,
                              pylon_camera_sync::AckCam::Response &res);
  void handleSlaveBeginTS(const std_msgs::UInt64::ConstPtr &timestamp);

  // ros params
  ros::NodeHandle m_nh;
  ros::ServiceServer m_exist_srv;
  ros::ServiceServer m_sync_srv;
  ros::Publisher m_timestamp_pub;
  ros::Publisher m_connection_pub;
  ros::Subscriber m_timestamp_sub;

  std::string m_device_id;
  std::vector<std::string> m_device_ids;

  int m_sync_count = 0;
  bool m_master_cam = false;
  bool m_start_grab = false;
  uint64_t m_begin_ts = 0;

  boost::shared_ptr<CameraManager> m_camera_manager = nullptr;
};

#endif  // CAMERAPROCESS_H