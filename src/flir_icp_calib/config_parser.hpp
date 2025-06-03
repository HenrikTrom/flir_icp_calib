#pragma once
#include "cpp_utils/jsontools.h"
#include "config.h"

namespace flir_icp_calib {

struct config_transform{

    bool online = true;
    bool log = true;
    std::string load_path_offline = "";
    std::string camera_settings_file = "";
    int aruco_dictionary = -1;
    int h = -1;
    int w = -1;
    int min_markers = 1000;
    std::string calibration_file = "";
    uint16_t min_cameras_per_point = 1000;
    std::string local_pattern_file = "";
    float max_reprojection_error_images = 0.f;
    float max_reprojection_error_robot = 0.f;
    std::string parent_frame_id = "";
    std::string child_frame_id = "";

};

bool load_transform_config(const std::string &cfg_name, config_transform &cfg_transform);

}