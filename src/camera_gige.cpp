#include "camera_gige.hpp"

#include <utility>

#include "encoding_conversions.h"

CameraGigE::CameraGigE(std::string device_id)
    : m_device_id(std::move(device_id)) {}

CameraGigE::~CameraGigE() { m_cam.reset(); }

bool CameraGigE::init() {
  if (!instantiateCamera()) {
    return false;
  }
  if (!enableChunks()) {
    return false;
  }
  if (!setupIEEE1588()) {
    return false;
  }

  return true;
}

/*
 * Instantiate and open the specific camera.
 */
bool CameraGigE::instantiateCamera() {
  int count = 0;
  int max_try = 3;
  Pylon::CTlFactory &tl_factory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t lst_devices;
  tl_factory.EnumerateDevices(lst_devices);

  while (count < 3) {
    if (!lst_devices.empty()) {
      for (int i = 0; i < lst_devices.size(); i++) {
        std::string device_user_id_found(lst_devices[i].GetUserDefinedName());
        // Match cameras id
        if (device_user_id_found == m_device_id) {
          // Instantiate device
          Pylon::IPylonDevice *device = tl_factory.CreateDevice(lst_devices[i]);
          // Instantiate GigE Camera
          m_cam.reset(new Pylon::CBaslerGigEInstantCamera(device));

          try {
            m_cam->Open();
          } catch (const GenICam::RuntimeException &e) {
            ROS_ERROR_STREAM("[" << m_device_id.c_str() << "] " << e.what());
            return false;
          }

          ROS_DEBUG("Found camera %s",
                    m_cam->GetDeviceInfo().GetFriendlyName().c_str());
          return true;
        }
      }
    }
    count++;
  }
  ROS_ERROR("Device \"%s\" not found", m_device_id.c_str());
  return false;
}

bool CameraGigE::applyStartupSettings(const PylonCameraParameter &parameters) {
  this->loadConfiguration();

  /* Thresholds for the AutoExposure Functions:
   *  - lower limit can be used to get rid of changing light conditions
   *    due to 50Hz lamps (-> 20ms cycle duration)
   *  - upper limit is to prevent motion blur
   */
  m_exposure_lower_lim = this->exposureTime().GetMin();
  m_exposure_lower_lim = std::max(this->autoExposureTimeLowerLimit().GetValue(),
                                  this->exposureTime().GetMin());

  // Check if the exposure timeout is set in the config/*.yaml file.
  if (parameters.auto_exp_upper_lim_ != 0) {
    m_exposure_upper_lim =
        std::min(parameters.auto_exp_upper_lim_, this->exposureTime().GetMax());
  }
  // Otherwise load it from the config/camera_config.pfs file.
  else {
    m_exposure_upper_lim =
        std::min(this->autoExposureTimeUpperLimit().GetValue(),
                 this->exposureTime().GetMax());
  }

  this->autoExposureTimeLowerLimit().SetValue(m_exposure_lower_lim);
  this->autoExposureTimeUpperLimit().SetValue(m_exposure_upper_lim);

  this->autoGainLowerLimit().SetValue(this->gain().GetMin());
  this->autoGainUpperLimit().SetValue(this->gain().GetMax());

  // Enable Exposure Auto by setting the operating mode to Continuous
  m_cam->ExposureAuto.SetValue(
      Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off);
  m_cam->ExposureAuto.SetValue(
      Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous);

  // raise inter-package delay (GevSCPD) for solving error:
  // 'the image buffer was incompletely grabbed'
  // also in ubuntu settings -> network -> options -> MTU Size
  // from 'automatic' to 3000 if card supports it
  // Raspberry PI has MTU = 1500, max value for some cards: 9000
  m_cam->GevSCPSPacketSize.SetValue(parameters.mtu_size_);

  // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
  // inter package delay in ticks (? -> mathi said in nanosec) -> prevent lost
  // frames package size * n_cams + 5% overhead = inter package size int n_cams
  // = 1; int inter_package_delay_in_ticks = n_cams * imageSize() * 1.05;
  m_cam->GevSCPD.SetValue(parameters.inter_pkg_delay_);

  return true;
}

// TODO: Link the Configuration path in the config 'ac_#.yaml' file.
bool CameraGigE::loadConfiguration() {
  std::string path =
      ros::package::getPath("pylon_camera_sync") + CAMERA_CONFIG_FILE;

  // Read the content of the file back to the camera's node map with validation
  // on
  try {
    Pylon::CFeaturePersistence::Load(path.c_str(), &m_cam->GetNodeMap(), true);

    return true;
  } catch (Pylon::GenericException &e) {
    // Error handling
    ROS_ERROR("[%s] %s", m_device_id.c_str(), e.GetDescription());
    return false;
  }
}

/*
 * Enable Chunk features, this allow the camera to generate specific information
 * such as the timestamp at the moment of the triggering.
 */
bool CameraGigE::enableChunks() {
  m_cam->Open();
  // Enable chunks in general.
  if (GenApi::IsWritable(m_cam->ChunkModeActive)) {
    m_cam->ChunkModeActive.SetValue(true);
  } else {
    ROS_ERROR("[%s] The camera doesn't support chunks features",
              m_device_id.c_str());
    return false;
  }
  // Enable time stamp chunks.
  m_cam->ChunkSelector.SetValue(Basler_GigECamera::ChunkSelector_Timestamp);
  m_cam->ChunkEnable.SetValue(true);

  // Enable CRC checksum chunks.
  m_cam->ChunkSelector.SetValue(Basler_GigECamera::ChunkSelector_PayloadCRC16);
  m_cam->ChunkEnable.SetValue(true);

  ROS_DEBUG("[%s] Chunk features enabled", m_device_id.c_str());
  return true;
}

/*
 * Enable IEEE1588 protocol (PTP) and sync to the master clock
 * within a certain offset.
 */
bool CameraGigE::setupIEEE1588() {
  m_cam->Open();
  if (GenApi::IsAvailable(m_cam->GevIEEE1588)) {
    m_cam->GevIEEE1588.SetValue(true);
    m_cam->GevIEEE1588DataSetLatch.Execute();
    ROS_INFO("[%s] PTP connection: clock ID: %ld, offset: %ld",
             m_device_id.c_str(),
             m_cam->GevIEEE1588ClockId.GetValue(),
             m_cam->GevIEEE1588OffsetFromMaster.GetValue());
    return true;
  }
  ROS_ERROR("[%s] IEEE1588 mode not available", m_device_id.c_str());
  return false;
}

/*
 * Configure the camera in Synchronous Free Run mode.
     cam.Attach(tlFactory.CreateDevice(devices[i]))
 */
bool CameraGigE::enableFreeRunMode() {
  // Make sure that the Frame Start trigger is set to Off to enable free run
  m_cam->TriggerSelector.SetValue(
      Basler_GigECamera::TriggerSelectorEnums::TriggerSelector_FrameStart);
  m_cam->TriggerMode.SetValue(
      Basler_GigECamera::TriggerModeEnums::TriggerMode_Off);

  // Assign the beginning grabbing time
  m_cam->SyncFreeRunTimerStartTimeLow.SetValue(0);
  m_cam->SyncFreeRunTimerStartTimeHigh.SetValue(0);

  // Specify a trigger rate frames per second
  m_cam->SyncFreeRunTimerTriggerRateAbs.SetValue(FRAME_RATE);

  // Apply the changes
  m_cam->SyncFreeRunTimerUpdate.Execute();

  // Enable Synchronous Free Run
  m_cam->SyncFreeRunTimerEnable.SetValue(true);

  return true;
}

/*
 * Get current timestamp from the camera.
 */
uint64_t CameraGigE::getCurrTimeStamp() {
  if (m_cam != nullptr) {
    m_cam->GevTimestampControlLatch.Execute();
    return m_cam->GevTimestampValue.GetValue();
  }

  return 0;
}

bool CameraGigE::startGrabbing(const PylonCameraParameter &parameters) {
  m_available_image_encodings = detectAvailableImageEncodings();
  if (static_cast<int>(
          !(setImageEncoding(parameters.imageEncoding()).find("done") !=
            std::string::npos)) != 0) {
    ROS_ERROR("[%s] Image encoding '%s' not available",
              m_device_id.c_str(),
              parameters.imageEncoding().c_str());
    return false;
  }

  // Starts grabbing for all camera
  m_cam->StartGrabbing();

  m_device_id = m_cam->DeviceUserID.GetValue();
  m_img_rows = static_cast<size_t>(m_cam->Height.GetValue());
  m_img_cols = static_cast<size_t>(m_cam->Width.GetValue());
  m_img_size_byte = m_img_cols * m_img_rows * imagePixelDepth();

  // Set the grabbing timeout bigger of 0.5 of the exposure time.
  m_grab_timeout = static_cast<float>(m_exposure_upper_lim * 1.05);

  // grab one image to be sure that the communication is successful
  Pylon::CBaslerGigEGrabResultPtr grab_result;
  grab(grab_result);
  if (grab_result.IsValid()) {
    m_is_ready = true;
    return true;
  }
  ROS_ERROR("[%s] Initial grab invalid, camera not ready", m_device_id.c_str());
  return false;
}

bool CameraGigE::stopGrabbing() {
  m_cam->StopGrabbing();
  m_cam->Close();
  return true;
}

CameraGigE::GrabStatus CameraGigE::grab(
    Pylon::CBaslerGigEGrabResultPtr &ptr_grab_result) {
  try {
    m_cam->RetrieveResult(
        m_grab_timeout, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);

    // Check the integrity of the buffer first.
    if (ptr_grab_result->HasCRC() && !ptr_grab_result->CheckCRC()) {
      throw RUNTIME_EXCEPTION("Image was damaged!");
      return CameraGigE::GrabStatus::Error;
    }
    if (ptr_grab_result->GrabSucceeded()) {
      return CameraGigE::GrabStatus::Successful;
    }  // If a buffer has been incompletely grabbed, the network bandwidth is
    // possibly insufficient for transferring multiple images simultaneously.
    // See note above c_maxCamerasToUse.
    ROS_ERROR("Error: %lu - %s",
              (unsigned long)ptr_grab_result->GetErrorCode(),
              ptr_grab_result->GetErrorDescription().c_str());
    return CameraGigE::GrabStatus::Error;

  } catch (const GenICam::TimeoutException &e) {
    ROS_ERROR_STREAM("[" << m_device_id.c_str() << "] " << e.what());
    return CameraGigE::GrabStatus::Error;
  } catch (const GenICam::LogicalErrorException &e) {
    // Usually when a Logical Error Exception occurs is not a big deal and we
    // don't wonw display any errors, but just a warning.
    ROS_DEBUG_STREAM("[" << m_device_id.c_str() << "] " << e.what());
    return CameraGigE::GrabStatus::LogicalErrorException;
  }
}

bool CameraGigE::grab(std::vector<uint8_t> &image, ros::Time &stamp) {
  Pylon::CBaslerGigEGrabResultPtr ptr_grab_result;
  CameraGigE::GrabStatus grab_status = grab(ptr_grab_result);

  if (grab_status == CameraGigE::GrabStatus::Error) {
    ROS_ERROR("[%s] Grab was not successful", m_device_id.c_str());
    return false;
  }
  if (grab_status == CameraGigE::GrabStatus::LogicalErrorException) {
    ROS_WARN("[%s] Logical error exception", m_device_id.c_str());
    return false;
  }
  const uint8_t *pImageBuffer =
      reinterpret_cast<uint8_t *>(ptr_grab_result->GetBuffer());

  // ------------------------------------------------------------------------
  // Bit shifting
  // ------------------------------------------------------------------------
  // In case of 12 bits we need to shift the image bits 4 positions to the left
  // Dynamically allocated to avoid heap size error
  std::string ros_enc = currentROSEncoding();
  auto *shift_array = new uint16_t[m_img_size_byte / 2];
  std::string gen_api_encoding(m_cam->PixelFormat.ToString().c_str());
  if (encoding_conversions::is_12_bit_ros_enc(ros_enc) &&
      (gen_api_encoding == "BayerRG12" || gen_api_encoding == "BayerBG12" ||
       gen_api_encoding == "BayerGB12" || gen_api_encoding == "BayerGR12" ||
       gen_api_encoding == "Mono12")) {
    const uint16_t *convert_bits =
        reinterpret_cast<uint16_t *>(ptr_grab_result->GetBuffer());
    for (int i = 0; i < m_img_size_byte / 2; i++) {
      shift_array[i] = convert_bits[i] << 4;
    }
    image.assign((uint8_t *)shift_array,
                 (uint8_t *)shift_array + m_img_size_byte);
  } else {
    image.assign(pImageBuffer, pImageBuffer + m_img_size_byte);
  }
  delete[] shift_array;

  pImageBuffer = (uint8_t *)ptr_grab_result->GetBuffer();
  if (GenApi::IsReadable(ptr_grab_result->ChunkTimestamp)) {
    uint64_t chunckTimestampCopy = ptr_grab_result->ChunkTimestamp.GetValue();
    stamp.fromNSec(chunckTimestampCopy);
  } else {
    ROS_WARN("[%s] Error: unable to read the time stamp from camera",
             m_device_id.c_str());
    return false;
  }

  return true;
}

int CameraGigE::imagePixelDepth() {
  int pixel_depth(0);
  try {
    // pylon PixelSize already contains the number of channels
    // the size is given in bit, wheras ROS provides it in byte
    pixel_depth = m_cam->PixelSize.GetIntValue() / 8;
  } catch (const GenICam::GenericException &e) {
    ROS_ERROR_STREAM("An exception while reading image pixel size occurred: "
                     << e.GetDescription());
  }
  return pixel_depth;
}

std::string CameraGigE::currentROSEncoding() {
  std::string gen_api_encoding(m_cam->PixelFormat.ToString().c_str());
  std::string ros_encoding;
  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding)) {
    // std::stringstream ss;
    // ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding << "'
    // found! This is bad because this case should never occur!"; throw
    // std::runtime_error(ss.str());
    ROS_ERROR_STREAM("No ROS equivalent to GenApi encoding");
    m_cam->StopGrabbing();
    setImageEncoding(gen_api_encoding);
    m_cam->StartGrabbing();
    // return "NO_ENCODING";
  }
  return ros_encoding;
}

std::vector<std::string> CameraGigE::detectAvailableImageEncodings() {
  std::vector<std::string> available_encodings;
  GenApi::INodeMap &node_map = m_cam->GetNodeMap();
  GenApi::CEnumerationPtr img_encoding_enumeration_ptr(
      node_map.GetNode("PixelFormat"));
  GenApi::NodeList_t feature_list;
  img_encoding_enumeration_ptr->GetEntries(feature_list);
  std::stringstream ss;
  ss << "Cam supports the following [GenAPI|ROS] image encodings: ";
  for (auto &it : feature_list) {
    if (GenApi::IsAvailable(it)) {
      GenApi::CEnumEntryPtr enum_entry(it);
      std::string encoding_gen_api = enum_entry->GetSymbolic().c_str();
      std::string encoding_ros("NO_ROS_EQUIVALENT");
      encoding_conversions::genAPI2Ros(encoding_gen_api, encoding_ros);
      ss << "['" << encoding_gen_api << "'|'" << encoding_ros << "'] ";
      available_encodings.push_back(encoding_gen_api);
    }
  }

  ROS_DEBUG_STREAM(ss.str().c_str());

  return available_encodings;
}

std::string CameraGigE::setImageEncoding(const std::string &ros_encoding) {
  bool is_16bits_available = false;
  bool is_encoding_available = false;
  std::string gen_api_encoding;
  // An additional check to select the correct basler encoding, as ROS 16-bits
  // encoding will cover both Basler 12-bits and 16-bits encoding
  if (ros_encoding == "bayer_rggb16" || ros_encoding == "bayer_bggr16" ||
      ros_encoding == "bayer_gbrg16" || ros_encoding == "bayer_grbg16") {
    for (const std::string &enc : m_available_image_encodings) {
      if (enc == "BayerRG16" || enc == "BayerBG16" || enc == "BayerGB16" ||
          enc == "BayerGR16" || enc == "Mono16") {
        is_16bits_available = true;
        break;
      }
    }
  }
  bool conversion_found = encoding_conversions::ros2GenAPI(
      ros_encoding, gen_api_encoding, is_16bits_available);
  if (!ros_encoding.empty()) {
    for (const std::string &enc : m_available_image_encodings) {
      if ((gen_api_encoding == enc) && conversion_found) {
        is_encoding_available = true;
        break;
      }
    }
    if (!is_encoding_available) {
      return "Error: unsporrted/unknown image format";
    }
  }
  if (!conversion_found) {
    if (ros_encoding.empty()) {
      ROS_WARN_STREAM("No image encoding provided. Will use 'mono8' or "
                      << "'rgb8' as fallback!");
    } else {
      ROS_ERROR_STREAM(
          "Can't convert ROS encoding '"
          << ros_encoding
          << "' to a corresponding GenAPI encoding! Will use 'mono8' or "
          << "'rgb8' as fallback!");
    }
    bool fallback_found = false;
    for (const std::string &enc : m_available_image_encodings) {
      if (enc == "Mono8" || enc == "RGB8") {
        fallback_found = true;
        gen_api_encoding = enc;
        break;
      }
    }
    if (!fallback_found) {
      ROS_ERROR_STREAM("Couldn't find a fallback solution!");
      return "Error: Couldn't find a fallback solution!";
    }
  }

  bool supports_desired_encoding = false;
  for (const std::string &enc : m_available_image_encodings) {
    supports_desired_encoding = (gen_api_encoding == enc);
    if (supports_desired_encoding) {
      break;
    }
  }
  if (!supports_desired_encoding) {
    ROS_WARN_STREAM("Camera does not support the desired image pixel "
                    << "encoding '" << ros_encoding << "'!");
    return "Error : Camera does not support the desired image pixel";
  }
  try {
    if (GenApi::IsAvailable(m_cam->PixelFormat)) {
      GenApi::INodeMap &node_map = m_cam->GetNodeMap();
      GenApi::CEnumerationPtr(node_map.GetNode("PixelFormat"))
          ->FromString(gen_api_encoding.c_str());
      return "done";
    }
    ROS_WARN_STREAM("Camera does not support variable image pixel "
                    << "encoding!");
    return "Error : Camera does not support variable image pixel";

  } catch (const GenICam::GenericException &e) {
    ROS_ERROR_STREAM("An exception while setting target image encoding to '"
                     << ros_encoding << "' occurred: " << e.GetDescription());
    return e.GetDescription();
  }
}

const std::string &CameraGigE::deviceUserID() { return m_device_id; }

const size_t &CameraGigE::getImageRows() const { return m_img_rows; }

const size_t &CameraGigE::getImageCols() const { return m_img_cols; }

const size_t &CameraGigE::getImageSize() const { return m_img_size_byte; }

size_t CameraGigE::getCurrentBinningX() {
  if (GenApi::IsAvailable(m_cam->BinningHorizontal)) {
    return static_cast<size_t>(m_cam->BinningHorizontal.GetValue());
  }
  return 1;
}

size_t CameraGigE::getCurrentBinningY() {
  if (GenApi::IsAvailable(m_cam->BinningVertical)) {
    return static_cast<size_t>(m_cam->BinningVertical.GetValue());
  }
  return 1;
}

GenApi::IFloat &CameraGigE::exposureTime() {
  if (GenApi::IsAvailable(m_cam->ExposureTimeAbs)) {
    return m_cam->ExposureTimeAbs;
  }
  throw std::runtime_error(
      "Error while accessing ExposureTimeAbs in CameraGigE");
}

GenApi::IFloat &CameraGigE::autoExposureTimeLowerLimit() {
  if (GenApi::IsAvailable(m_cam->AutoExposureTimeAbsLowerLimit)) {
    return m_cam->AutoExposureTimeAbsLowerLimit;
  }
  throw std::runtime_error(
      "Error while accessing AutoExposureTimeAbsLowerLimit in CameraGigE");
}

GenApi::IFloat &CameraGigE::autoExposureTimeUpperLimit() {
  if (GenApi::IsAvailable(m_cam->AutoExposureTimeAbsUpperLimit)) {
    return m_cam->AutoExposureTimeAbsUpperLimit;
  }
  throw std::runtime_error(
      "Error while accessing AutoExposureTimeAbsUpperLimit in CameraGigE");
}

GenApi::IInteger &CameraGigE::autoGainLowerLimit() {
  if (GenApi::IsAvailable(m_cam->AutoGainRawLowerLimit)) {
    return m_cam->AutoGainRawLowerLimit;
  }
  throw std::runtime_error(
      "Error while accessing AutoGainRawLowerLimit in CameraGigE");
}

GenApi::IInteger &CameraGigE::autoGainUpperLimit() {
  if (GenApi::IsAvailable(m_cam->AutoGainRawUpperLimit)) {
    return m_cam->AutoGainRawUpperLimit;
  }
  throw std::runtime_error(
      "Error while accessing AutoGainRawUpperLimit in CameraGigE");
}

GenApi::IFloat &CameraGigE::resultingFrameRate() {
  if (GenApi::IsAvailable(m_cam->ResultingFrameRateAbs)) {
    return m_cam->ResultingFrameRateAbs;
  }
  throw std::runtime_error(
      "Error while accessing ResultingFrameRateAbs in CameraGigE");
}

GenApi::IInteger &CameraGigE::autoTargetBrightness() {
  if (GenApi::IsAvailable(m_cam->AutoTargetValue)) {
    return m_cam->AutoTargetValue;
  }
  throw std::runtime_error(
      "Error while accessing AutoTargetValue in CameraGigE");
}

GenApi::IInteger &CameraGigE::gain() {
  if (GenApi::IsAvailable(m_cam->GainRaw)) {
    return m_cam->GainRaw;
  }
  throw std::runtime_error("Error while accessing GainRaw in PylonGigECamera");
}
