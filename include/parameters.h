#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>
#include <vector>

static const std::vector<std::string> DEVICES_ID{"ac-0", "ac-1"};

static const std::string TOPIC_IMG_RAW = "image_raw";
static const std::string TOPIC_BEGIN_TS = "/pylon_camera_sync/begin_timestamp";
static const std::string TOPIC_CONNECTION = "/pylon_camera_sync/connection";
static const std::string SERVICE_SOVEREIGNTY = "/pylon_camera_sync/sovereignty";
static const std::string SERVICE_SYNC = "/pylon_camera_sync/sync";
static const std::string CAMERA_CONFIG_FILE = "/config/camera_config.pfs";

// Need to be higher or equal to 3 in order to
// give time to the cameras to load the sync file.
static const int WAIT_EXIST_CAM = 3;  // 3s

static const uint64_t BEGIN_TS_OFFSET = 1e9;  // 1s (in ns)
static const uint64_t ONESEC_NS = 1e9;        // 1s (in ns)

static const int PTP_OFFSET_LIMIT = 100000;  // 0.1ms (in ns)
static const int PTP_ATTEMPTS_LIMIT = 7;
static const double PTP_WAIT_SLAVE_TIME = 2;     // 5s
static const double PTP_WAIT_OFFSET_TIME = 2.5;  // 2.5s

static const int FRAME_RATE = 19;

#endif  // PARAMETERS_H
