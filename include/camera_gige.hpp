#ifndef CAMERAGIGE_H
#define CAMERAGIGE_H

#include <GenApi/GenApi.h>
#include <pylon/PylonIncludes.h>
#include <pylon/PylonUtilityIncludes.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>
#include <pylon/gige/GigETransportLayer.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "camera_parameter.hpp"
#include "parameters.h"

class CameraGigE {
 public:
  enum GrabStatus {
    Successful = 0,
    Error = 1,
    LogicalErrorException = 2,
  };
  CameraGigE(std::string device_id);
  ~CameraGigE();
  bool init();
  bool instantiateCamera();
  bool applyStartupSettings(const PylonCameraParameter &parameters);
  bool enableFreeRunMode();
  uint64_t getCurrTimeStamp();
  bool startGrabbing(const PylonCameraParameter &parameters);
  bool stopGrabbing();
  bool grab(std::vector<uint8_t> &image, ros::Time &stamp);

  const std::string &deviceUserID();
  const size_t &getImageRows() const;
  const size_t &getImageCols() const;
  const size_t &getImageSize() const;
  size_t getCurrentBinningX();
  size_t getCurrentBinningY();

  std::string currentROSEncoding();
  int imagePixelDepth();
  bool enableChunks();
  bool setupIEEE1588();

 private:
  bool loadConfiguration();
  GrabStatus grab(Pylon::CBaslerGigEGrabResultPtr &ptr_grab_result);
  std::vector<std::string> detectAvailableImageEncodings();
  std::string setImageEncoding(const std::string &ros_encoding);
  GenApi::IFloat &exposureTime();
  GenApi::IFloat &autoExposureTimeLowerLimit();
  GenApi::IFloat &autoExposureTimeUpperLimit();
  GenApi::IInteger &autoGainLowerLimit();
  GenApi::IInteger &autoGainUpperLimit();
  GenApi::IFloat &resultingFrameRate();
  GenApi::IInteger &autoTargetBrightness();
  GenApi::IInteger &gain();

  // Util methods
  static void u64ToHighLow(uint64_t value, uint32_t &hi, uint32_t &lo);

  /*
   * Pylon Camera GigE shared pointer.
   */
  boost::shared_ptr<Pylon::CBaslerGigEInstantCamera> m_cam;

  /**
   * Vector that contains the available image_encodings the camera supports.
   * The strings describe the GenAPI encoding.
   */
  std::vector<std::string> m_available_image_encodings;

  /**
   * The DeviceUserID of the camera.
   */
  std::string m_device_id;

  /**
   * Number of image rows.
   */
  size_t m_img_rows{};

  /**
   * Number of image columns.
   */
  size_t m_img_cols{};

  /**
   * The size of the image in number of bytes.
   */
  size_t m_img_size_byte{};

  /**
   * The max time a single grab is allowed to take. This value should always
   * be greater then the max possible exposure time of the camera
   */
  float m_grab_timeout{};

  /**
   * Flag which is set in case that the grab-result-pointer of the first
   * acquisition contains valid data
   */
  bool m_is_ready{};

  /**
   * Lower and upper limit for the exposure time.
   */
  int64_t m_exposure_upper_lim{};
  int64_t m_exposure_lower_lim{};
};

#endif  // CAMERAGIGE_H