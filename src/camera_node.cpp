#include "camera_node.hpp"

using namespace std::chrono;

/*
 * Class constructor.
 */
CameraNode::CameraNode() : m_nh("~") {
  Pylon::PylonInitialize();
  m_camera_manager = boost::make_shared<CameraManager>();
  m_device_id = m_camera_manager->getDeviceUserId();

  if (!defineSovereignty()) {
    ros::shutdown();
    return;
  }

  if (!m_camera_manager->init()) {
    ros::shutdown();
    return;
  }

  if (!syncCameras()) {
    ros::shutdown();
    return;
  }

  if (!m_camera_manager->enableFreeRunMode()) {
    ros::shutdown();
    return;
  }

  run();
}

/*
 * Class destructor.
 */
CameraNode::~CameraNode() { m_camera_manager.reset(); }

/*
 * Each camera node will check wether to assume the master or slave role. The
 * master role comunicate the image grabbing starting timestamp to the slave
 * cameras. Since each camera node is run independently, the communication of
 * the time is fundamental and required for the Synchronous Free Run mode.
 */
bool CameraNode::defineSovereignty() {
  // #################################
  // MASTER ROLE
  // #################################
  // If the SOVEREIGNTY service do not exist, assume the master role.

  if (!ros::service::exists(SERVICE_SOVEREIGNTY, false)) {
    // Set the camera as master
    m_master_cam = true;

    // Define all the master services
    // -----------------------------------------------------------
    m_exist_srv = m_nh.advertiseService(
        SERVICE_SOVEREIGNTY, &CameraNode::handleMasterExistCamera, this);
    m_sync_srv = m_nh.advertiseService(
        SERVICE_SYNC, &CameraNode::handleMasterSyncCamera, this);

    m_timestamp_pub = m_nh.advertise<std_msgs::UInt64>(TOPIC_BEGIN_TS, 1);
    m_connection_pub = m_nh.advertise<std_msgs::Bool>(TOPIC_CONNECTION, 1);

    // -----------------------------------------------------------

    ROS_DEBUG("~ Listening for existing cameras");
    ROS_DEBUG("[%s] Role: master camera", m_device_id.c_str());

    // ==============================================================
    // 1) Wait the Acknowledgment of existence from the slave cameras
    // ==============================================================
    m_device_ids.push_back(m_device_id);

    ros::Rate rate(100);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();

    // Wait for a specific amount of time (WAIT_EXIST_CAM)
    ros::Duration offset(WAIT_EXIST_CAM);
    while (ros::ok() && curr < (begin + offset)) {
      ros::spinOnce();
      rate.sleep();
      curr = ros::Time::now();
    }

    // Shutting down existing advertise service
    m_exist_srv.shutdown();
  }

  // #################################
  // SLAVE ROLE
  // #################################

  else if (!m_master_cam) {
    // Define all the slave services
    // -----------------------------------------------------------
    m_timestamp_sub = m_nh.subscribe<std_msgs::UInt64>(
        TOPIC_BEGIN_TS, 1, &CameraNode::handleSlaveBeginTS, this);
    // -----------------------------------------------------------

    // ==============================================================
    // 1) Acknowledgment of existence to the Master camera
    // ==============================================================
    ros::ServiceClient client =
        m_nh.serviceClient<pylon_camera_sync::AckCam>(SERVICE_SOVEREIGNTY);

    pylon_camera_sync::AckCam srv_ack_exist;
    srv_ack_exist.request.device_id = m_device_id;

    if (client.call(srv_ack_exist) && srv_ack_exist.response.success) {
      ROS_DEBUG("[%s] Role: slave camera", m_device_id.c_str());
    } else {
      ROS_ERROR("[%s] Failed to call service %s",
                m_device_id.c_str(),
                SERVICE_SOVEREIGNTY.c_str());
      return false;
    }
  }

  return true;
}

/*
 * Communication of the image grabbing beginning timestamp to the
 * slave cameras.
 */
bool CameraNode::syncCameras() {
  // #################################
  // MASTER ROLE
  // #################################
  // The Master waits that all the cameras are sync
  if (m_master_cam) {
    // ==============================================================
    // 2) Wating the synchronization from all the registered Slaves
    // ==============================================================

    // Increment the number of sync cameras (here only the master)
    m_sync_count++;

    ros::Rate rate(100);
    while (ros::ok() && (m_sync_count < (m_device_ids.size()))) {
      ros::spinOnce();
      rate.sleep();
    }

    // Shutting down sync advertise service
    m_sync_srv.shutdown();

    // ==============================================================
    // 3) Communication of the beguining timestamp to all the slaves
    // ==============================================================

    // Get current timestamp and add an offset of time
    m_begin_ts = m_camera_manager->getCurrTimeStamp() + BEGIN_TS_OFFSET;

    // Create and comunicate the beginning timestamp message
    std_msgs::UInt64 message;
    message.data = m_begin_ts;
    m_timestamp_pub.publish(message);

    ROS_INFO("~ All cameras are synchronized, start grabbing in %ld seconds",
             (BEGIN_TS_OFFSET / ONESEC_NS));
  }

  // #################################
  // SLAVE ROLE
  // #################################
  // The Slave is comunicating its sync status to the master and
  // wating to receive the beginning timestamp.
  else {
    // ==============================================================
    // 2) Communication of the synchronization to Master
    // ==============================================================
    ros::ServiceClient client =
        m_nh.serviceClient<pylon_camera_sync::AckCam>(SERVICE_SYNC);

    pylon_camera_sync::AckCam srv;
    srv.request.device_id = m_device_id;

    if (!client.call(srv)) {
      ROS_ERROR("[%s] Failed to call service %s",
                m_device_id.c_str(),
                SERVICE_SYNC.c_str());
      return false;
    }

    ROS_DEBUG("[%s] Communicated the synchronization to master camera",
              m_device_id.c_str());

    // ==============================================================
    // 3) Wating the beginning timestamp from the master
    // ==============================================================

    ros::Rate rate(100);
    while (ros::ok() && !m_start_grab) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  return true;
}

/*
 * Service handler that receive the communication of
 * the camera ID from all the slave cameras.
 */
bool CameraNode::handleMasterExistCamera(
    pylon_camera_sync::AckCam::Request &req,
    pylon_camera_sync::AckCam::Response &res) {
  // Save existing cam device
  m_device_ids.push_back(req.device_id);
  res.success = true;

  ROS_DEBUG("~ Camera \"%s\" detected", req.device_id.c_str());
  return true;
}

/*
 * Service handler that register the number of slave
 * cameras synced and ready to start grabbing images.
 */
bool CameraNode::handleMasterSyncCamera(
    pylon_camera_sync::AckCam::Request &req,
    pylon_camera_sync::AckCam::Response &res) {
  std::string device_id = req.device_id;

  if (std::count(m_device_ids.begin(), m_device_ids.end(), device_id) == 0) {
    res.success = false;
    ROS_ERROR("~ Unknown camera \"%s\" is trying to synchronize",
              req.device_id.c_str());
    return false;
  }

  // Increment the number of sync cameras (here only the slave)
  m_sync_count++;

  // Define a success communication
  res.success = true;
  ROS_DEBUG("~ Camera \"%s\" is sync", req.device_id.c_str());

  return true;
}

void CameraNode::handleSlaveBeginTS(
    const std_msgs::UInt64::ConstPtr &timestamp) {
  m_begin_ts = timestamp->data;
  m_start_grab = true;
  ROS_DEBUG("~ Beginning timestamp received from master (%ld)", m_begin_ts);
}

/*
 * Run the process.
 */
void CameraNode::run() {
  if (!m_camera_manager->startGrabbing()) {
    ROS_INFO("[%s] Error in starting grabbing images", m_device_id.c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("[%s] Starting grabbing images", m_device_id.c_str());

  // The master camera publish a ack message notifing
  // that the cameras are starting to grab images.
  if (m_master_cam) {
    std_msgs::Bool message;
    message.data = true;
    m_connection_pub.publish(message);
  }

  bool status = true;
  while (ros::ok() && status) {
    status = m_camera_manager->spin();
    ros::spinOnce();
  }

  if (!m_camera_manager->stopGrabbing()) {
    ros::shutdown();
    return;
  }
}
