#include "config_parser.hpp"

namespace flir_icp_calib{

bool load_transform_config(const std::string &cfg_path, config_transform &cfg_transform){
    std::string schemepath = std::string(CONFIG_DIR)+"/TransformsSettings.Scheme.json";
    rapidjson::Document doc;
    if(!cpp_utils::load_json_with_schema(cfg_path, schemepath, 65536, doc)){
        std::string msg = "Could not load pipeline config";
        spdlog::error(msg);
        throw std::runtime_error(msg);
        return false;
    }
    cfg_transform.online = doc["online"].GetBool();
    cfg_transform.load_path_offline = doc["load_path_offline"].GetString();
    cfg_transform.camera_settings_file = doc["camera_settings_file"].GetString();
    cfg_transform.aruco_dictionary = doc["aruco_dictionary"].GetInt();
    cfg_transform.h = doc["h"].GetInt();
    cfg_transform.w = doc["w"].GetInt();
    cfg_transform.min_markers = doc["min_markers"].GetInt();
    cfg_transform.calibration_file = doc["calibration_file"].GetString();
    cfg_transform.min_cameras_per_point = static_cast<uint16_t>(doc["min_cameras_per_point"].GetUint());
    cfg_transform.local_pattern_file = doc["local_pattern_file"].GetString();
    cfg_transform.max_reprojection_error_images = doc["max_reprojection_error_images"].GetFloat();
    cfg_transform.max_reprojection_error_robot = doc["max_reprojection_error_robot"].GetFloat();
    cfg_transform.parent_frame_id = doc["parent_frame_id"].GetString();
    cfg_transform.child_frame_id = doc["child_frame_id"].GetString();

    std::string msg = std::string("Transform Interface Settings: \n")+
        std::string("\t- online:\t{}\n\t- online:\t{}\n\t- log:\t{}\n\t- load_path_offline:\t{}\n\t- camera_settings_file h:\t{}\n")+
        std::string("\t- aruco_dictionary w:\t{}\n\t- h:\t{}\n\t- w:\t{}\n")+
        std::string("\t- min_markers w:\t{}\n\t- calibration_file:\t{}\n\t- min_cameras_per_point:\t{}\n")+
        std::string("\t- local_pattern_file w:\t{}\n\t- max_reprojection_error_images:\t{}\n\t- max_reprojection_error_robot:\t{}\n")+
        std::string("\t- parent_frame_id w:\t{}\n\t- child_frame_id:\t{}");
    spdlog::info(
        msg,
        cfg_transform.online,
        cfg_transform.log,
        cfg_transform.load_path_offline,
        cfg_transform.camera_settings_file,
        cfg_transform.aruco_dictionary,
        cfg_transform.h,
        cfg_transform.w,
        cfg_transform.min_markers,
        cfg_transform.calibration_file,
        cfg_transform.min_cameras_per_point,
        cfg_transform.local_pattern_file,
        cfg_transform.max_reprojection_error_images,
        cfg_transform.max_reprojection_error_robot,
        cfg_transform.parent_frame_id,
        cfg_transform.child_frame_id
    );

    return true;
}

}