#include "../flir_icp_calib/interface.hpp"
#include "ros-node-interface/interface.hpp"


int main(int argc, char **argv) {
    if (!argc == 2){
        std::string msg = "Failed to parse Settings.json";
        spdlog::error(msg);
        throw std::runtime_error(msg);
        return 1;
    }
    std::string cfg_path = std::string(argv[1]);
    ros::init(argc, argv, "transform_interface_aruco");
    ros::NodeHandle nh("~");
    std::unique_ptr<flir_icp_calib::TransformInterfaceAruco> transform_interface_module;
    transform_interface_module.reset(
        new flir_icp_calib::TransformInterfaceAruco(
            nh, cfg_path
        )
    );
    ros_node_interface::BaseRosInterface<flir_icp_calib::TransformInterfaceAruco>transform_interface(
        std::move(transform_interface_module));
    std::cout << "Main thread exiting (transform_interface_aruco)..." << std::endl;

    return 0;
}