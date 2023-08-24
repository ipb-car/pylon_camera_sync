#include "camera_manager.hpp"

CameraManager::CameraManager()
    : m_nh("~"),

      m_it(new image_transport::ImageTransport(m_nh)),
      m_img_raw_pub(m_it->advertiseCamera(TOPIC_IMG_RAW, 1)),
      m_camera_info_manager(new camera_info_manager::CameraInfoManager(m_nh)) {
  // reading all necessary parameter to open the desired camera from the
  // ros-parameter-server. In case that invalid parameter values can be
  // detected, the interface will reset them to the default values.
  // These parameters furthermore contain the intrinsic calibration matrices,
  // in case they are provided
  m_camera_parameter_set.readFromRosParameterServer(m_nh);
  m_camera_gige.reset(new CameraGigE(m_camera_parameter_set.deviceUserID()));
}

CameraManager::~CameraManager() {
  m_camera_gige.reset();
  m_camera_info_manager.reset();
}

bool CameraManager::init() {
  if (!m_camera_gige->instantiateCamera()) {
    return false;
  }
  if (!m_camera_gige->applyStartupSettings(m_camera_parameter_set)) {
    return false;
  }
  if (!m_camera_gige->enableChunks()) {
    return false;
  }
  if (!m_camera_gige->setupIEEE1588()) {
    return false;
  }

  return true;
}

uint64_t CameraManager::getCurrTimeStamp() {
  return m_camera_gige->getCurrTimeStamp();
}

/*
 * Configure the camera in Synchronous Free Run mode.
 */
bool CameraManager::enableFreeRunMode() {
  return m_camera_gige->enableFreeRunMode();
}

bool CameraManager::startGrabbing() {
  if (!m_camera_gige->startGrabbing(m_camera_parameter_set)) {
    return false;
  }

  m_img_raw_msg.header.frame_id = m_camera_parameter_set.deviceUserID();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  m_img_raw_msg.encoding = m_camera_gige->currentROSEncoding();
  m_img_raw_msg.height = m_camera_gige->getImageRows();
  m_img_raw_msg.width = m_camera_gige->getImageCols();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  m_img_raw_msg.step = m_img_raw_msg.width * m_camera_gige->imagePixelDepth();

  if (!m_camera_info_manager->setCameraName(m_camera_gige->deviceUserID())) {
    // valid name contains only alphanumeric signs and '_'
    ROS_ERROR_STREAM("[" << m_camera_gige->deviceUserID()
                         << "] name not valid for camera_info_manager");
    return false;
  }

  // Initial setting of the CameraInfo-msg, assuming no calibration given
  sensor_msgs::CameraInfo initial_cam_info;
  setupInitialCameraInfo(initial_cam_info);
  m_camera_info_manager->setCameraInfo(initial_cam_info);
  if (m_camera_parameter_set.cameraInfoURL().empty() ||
      !m_camera_info_manager->validateURL(
          m_camera_parameter_set.cameraInfoURL())) {
    ROS_DEBUG(
        "CameraInfoURL needed for rectification! ROS-Param: "
        "'%s/camera_info_url' = '%s' is invalid!",
        m_nh.getNamespace().c_str(),
        m_camera_parameter_set.cameraInfoURL().c_str());
    ROS_DEBUG_STREAM("CameraInfoURL should have following style: "
                     << "'file:///full/path/to/local/file.yaml' or "
                     << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
    ROS_DEBUG("Will only provide distorted /image_raw images!");
  } else {
    ROS_ERROR(
        "[%s] Camera calibration mode not yet implemented, remove URL value",
        m_camera_gige->deviceUserID().c_str());
    // override initial camera info if the url is valid
    // if ( m_camera_info_manager->loadCameraInfo(
    //                         m_camera_parameter_set.cameraInfoURL()) )
    // {
    //     setupRectification();
    //     // set the correct tf frame_id
    //     CameraInfoPtr cam_info(new CameraInfo(
    //                                 m_camera_info_manager->getCameraInfo()));
    //     cam_info->header.frame_id = img_raw_msg_.header.frame_id;
    //     m_camera_info_manager->setCameraInfo(*cam_info);
    // }
    // else
    // {
    //     ROS_WARN("Will only provide distorted /image_raw images!");
    // }
  }

  return true;
}

bool CameraManager::stopGrabbing() {
  m_camera_gige->stopGrabbing();
  return true;
}

bool CameraManager::spin() {
  // get actual cam_info-object in every frame
  sensor_msgs::CameraInfoPtr cam_info(
      new sensor_msgs::CameraInfo(m_camera_info_manager->getCameraInfo()));

  // grab the image
  m_camera_gige->grab(m_img_raw_msg.data, m_img_raw_msg.header.stamp);

  // Assign timestamp to cam_info
  cam_info->header.stamp = m_img_raw_msg.header.stamp;

  // Publish via image_transport
  m_img_raw_pub.publish(m_img_raw_msg, *cam_info);

  return true;
}

void CameraManager::setupInitialCameraInfo(
    sensor_msgs::CameraInfo &cam_info_msg) {
  std_msgs::Header header;
  header.frame_id = m_camera_parameter_set.deviceUserID();
  header.stamp = ros::Time::now();

  // http://www.ros.org/reps/rep-0104.html
  // If the camera is uncalibrated, the matrices D, K, R, P should be left
  // zeroed out. In particular, clients may assume that K[0] == 0.0
  // indicates an uncalibrated camera.
  cam_info_msg.header = header;

  // The image dimensions with which the camera was calibrated. Normally
  // this will be the full camera resolution in pixels. They remain fix, even
  // if binning is applied
  cam_info_msg.height = m_camera_gige->getImageRows();
  cam_info_msg.width = m_camera_gige->getImageCols();

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficient.
  // Empty D and distortion_model indicate that the CameraInfo cannot be used
  // to rectify points or images, either because the camera is not calibrated
  // or because the rectified image was produced using an unsupported
  // distortion model, e.g. the proprietary one used by Bumblebee cameras
  // [http://www.ros.org/reps/rep-0104.html].
  cam_info_msg.distortion_model = "";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] D.
  cam_info_msg.D = std::vector<double>(5, 0.);

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]  --> 3x3 row-major matrix
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
  // using the focal lengths (fx, fy) and principal point (cx, cy).
  cam_info_msg.K.assign(0.0);

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  cam_info_msg.R.assign(0.0);

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix of the
  // processed (rectified) image. That is, the left 3x3 portion is the normal
  // camera intrinsic matrix for the rectified image. It projects 3D points
  // in the camera coordinate frame to 2D pixel coordinates using the focal
  // lengths (fx', fy') and principal point (cx', cy') - these may differ from
  // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
  // cameras will also have R = the identity and P[1:3,1:3] = K.
  // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
  // position of the optical center of the second camera in the first
  // camera's frame. We assume Tz = 0 so both cameras are in the same
  // stereo image plane. The first camera always has Tx = Ty = 0.
  // For the right (second) camera of a horizontal stereo pair,
  // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
  // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
  // rectified image is given by:
  // [u v w]' = P * [X Y Z 1]'
  //        x = u / w
  //        y = v / w
  //  This holds for both images of a stereo pair.
  cam_info_msg.P.assign(0.0);

  // Binning refers here to any camera setting which combines rectangular
  // neighborhoods of pixels into larger "super-pixels." It reduces the
  // resolution of the output image to (width / binning_x) x (height /
  // binning_y). The default values binning_x = binning_y = 0 is considered the
  // same as binning_x = binning_y = 1 (no subsampling).
  cam_info_msg.binning_x = m_camera_gige->getCurrentBinningX();
  cam_info_msg.binning_y = m_camera_gige->getCurrentBinningY();

  // Region of interest (subwindow of full camera resolution), given in full
  // resolution (unbinned) image coordinates. A particular ROI always denotes
  // the same window of pixels on the camera sensor, regardless of binning
  // settings. The default setting of roi (all values 0) is considered the same
  // as full resolution (roi.width = width, roi.height = height).
  cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

/**
 * TODO: implement rectification
 */
void CameraManager::setupRectification() {
  // if ( !img_rect_pub_ )
  // {
  //     img_rect_pub_ = new ros::Publisher(
  //                         nh_.advertise<sensor_msgs::Image>("image_rect",
  //                         1));
  // }

  // if ( !grab_imgs_rect_as_ )
  // {
  //     grab_imgs_rect_as_ =
  //         new GrabImagesAS(nh_,
  //                          "grab_images_rect",
  //                          boost::bind(
  //                             &PylonCameraNode::grabImagesRectActionExecuteCB,
  //                             this,
  //                             _1),
  //                          false);
  //     grab_imgs_rect_as_->start();
  // }

  // if ( !pinhole_model_ )
  // {
  //     pinhole_model_ = new image_geometry::PinholeCameraModel();
  // }

  // pinhole_model_->fromCameraInfo(m_camera_info_manager->getCameraInfo());
  // if ( !cv_bridge_img_rect_ )
  // {
  //     cv_bridge_img_rect_ = new cv_bridge::CvImage();
  // }
  // cv_bridge_img_rect_->header = img_raw_msg_.header;
  // cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
}

std::string CameraManager::getDeviceUserId() {
  return m_camera_parameter_set.deviceUserID();
}