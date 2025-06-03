#pragma once
#include "methods.hpp"
#include "visualization.hpp"
#include "dataIO.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include "ros-node-interface/interface.hpp"
#include "ros-node-interface/utils.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <flirmulticamera/FlirCamera.h>
#include <cpp_utils/utils.h>

using namespace rapidjson;
using Eigen::MatrixXd;


namespace flir_icp_calib{

/**
 * Interface for online experiments. Must be embedded in ros_node_interface::BaseRosInterface
 */
class TransformInterfaceAruco : public ros_node_interface::BaseRosInterfaceModule{
public:
    TransformInterfaceAruco(){};
    ~TransformInterfaceAruco(){};
    TransformInterfaceAruco(ros::NodeHandle &nh, const std::string &cfg_name);

    /**
     * Main method
     * Computes and publishes the transform, given the config
     */
    bool start();
    void Terminate();
    /**
     * Placeholder to keep main-thread busy.
     */
    void ThreadFunction(void);
    std::unique_ptr<std::thread> ThreadHandle;

    bool get_images(std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> &frame);

private:
    double pub_delay = 0.0001;
    config_transform cfg;
    MultiCameras cameras;
    std::vector<Eigen::VectorXf> triangulated_points;
    std::vector<int> all_ids;
    std::string child_frame_id, parent_frame_id;
    std::string calib_path;
    Eigen::MatrixXf transform;

    cv::aruco::ArucoDetector aruco_det;

    tf2_ros::StaticTransformBroadcaster broadcaster;

    std_srvs::Trigger req;

    keiko_msgs::ArucoMarkers3d det_arucomarkers3d;
    visualization_msgs::MarkerArray det_rviz;
    ros::Publisher pub_det_aruco, pub_det_rviz;
};

}
