#include "../flir_icp_calib/dataIO.hpp"
#include "ros-node-interface/interface.hpp"

using namespace flir_icp_calib;

int main(int argc, char **argv) {
    if (!argc == 2){
        std::string msg = "You must pass a path to a logged results folder as argument";
        spdlog::error(msg);
        throw std::runtime_error(msg);
        return 1;
    }
    std::string result_path = std::string(argv[1]);
    config_transform cfg;
    MultiCameras cameras;
    load_transform_config(result_path+"/TransformsSettings.json", cfg);
    if (!load_calibration(result_path+"/Calibration.json", cameras)){
        std::string msg = "Faild to load cameras";
        spdlog::error(msg);
        throw std::runtime_error(msg);
    };
    
    ros::init(argc, argv, "transform_interface_aruco");
    ros::NodeHandle nh("~");
    
    std::vector<int> ids_intersection;
    std::vector<Eigen::VectorXf> triangulated_points;
    Eigen::MatrixXf transform{4, 4};
    load_single_result(
        result_path+"/result.json", ids_intersection, 
        triangulated_points, transform
    );
    
    keiko_msgs::ArucoMarkers3d det_arucomarkers3d;
    visualization_msgs::MarkerArray det_rviz;
    ros::Publisher pub_det_aruco, pub_det_rviz;
    tf2_ros::StaticTransformBroadcaster broadcaster;
        pub_det_aruco=nh.advertise<keiko_msgs::ArucoMarkers3d>(
        std::string("aruco_corrdinates_")+cfg.parent_frame_id, 2);
    pub_det_rviz=nh.advertise<visualization_msgs::MarkerArray>(
        "det_aruco_markers", 2);
    publish_transforms(
        cameras, transform, broadcaster, 
        cfg.parent_frame_id, cfg.child_frame_id
    );

    std_msgs::ColorRGBA color;
    color.a=1;
    color.r=1;
    color.g=1;

    gen_aruco_msgs(
        triangulated_points, ids_intersection,
        cfg.parent_frame_id, color,
        det_arucomarkers3d, det_rviz
    );
    double pub_delay= 0.0001;
    spdlog::info("Publishing result...");
    while(ros::ok()){
        pub_det_aruco.publish(det_arucomarkers3d);
        pub_det_rviz.publish(det_rviz);
        ros::Duration(pub_delay).sleep();
    }

    return 0;
}