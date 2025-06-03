#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>

#include "rapidjson/document.h"

#include <Eigen/Dense>

#include <cpp_utils/jsontools.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include "config.h"

#include "methods.hpp"
#include "visualization.hpp"
#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <fstream>


using namespace rapidjson;

namespace flir_icp_calib{

/**
 * @brief Loads 3D ArUco marker corner data from a JSON calibration file.
 * 
 * This function parses a JSON file containing marker IDs as keys and 4x3 corner arrays
 * (in the robot base frame) as values. It returns the marker IDs and their corresponding 
 * 3D points in the order required for transformation estimation.
 * 
 * @param[in] calib_path Path to the calibration JSON file.
 * @param[out] ids Output vector of marker IDs found in the file.
 * @param[out] points_arucos Output vector of 3D points corresponding to each marker corner (4 points per marker).
 * @return true if the file exists and the data was successfully loaded, false otherwise.
 */
bool arucodata_from_json(
    std::string& calib_path,
    std::vector<int>& ids, std::vector<Eigen::VectorXf>& points_arucos
);

/**
 * @brief Serializes triangulation and transformation result into a JSON file.
 *
 * Writes the estimated transformation matrix, triangulated 3D points, and associated marker IDs
 * to a structured JSON file (`result.json`) at the specified path.
 *
 * @param[in] path Directory where the result file will be saved.
 * @param[in] ids Marker IDs corresponding to triangulated points.
 * @param[in] triangulated_points 3D coordinates of the triangulated marker corners (Nx3).
 * @param[in] transformation The computed 4x4 transformation matrix (camera to robot).
 * @return true if the file was written successfully, false otherwise.
 */
bool result_to_json(
    std::string path,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transformation
);

/**
 * @brief Saves the complete calibration result to a directory.
 *
 * This includes copying the camera calibration file, transform settings, and local pattern definition
 * into the specified directory, then saving the result JSON.
 *
 * @param[in] savepath Target directory for saving the result set.
 * @param[in] calibration_file Path to the camera calibration JSON file.
 * @param[in] local_pattern_file Path to the local 3D pattern JSON file.
 * @param[in] ids Marker IDs used during calibration.
 * @param[in] triangulated_points Corresponding triangulated 3D points.
 * @param[in] transform The final homogeneous transformation matrix.
 * @return true if all data was saved successfully, false otherwise.
 */
bool save_result(
    std::string &savepath,
    std::string &calibration_file,
    std::string &local_pattern_file,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transform
);

/**
 * @brief Publishes static ROS transforms for all cameras and the final robot transform.
 * 
 * Uses the given multi-camera setup and transformation matrix to broadcast static transforms
 * from the main camera frame to each secondary camera, and from the main camera frame to the robot.
 * 
 * @param[in] cameras Multi-camera structure with calibration info.
 * @param[in] transform 4x4 transformation matrix from camera to robot.
 * @param[in] broadcaster ROS static transform broadcaster.
 * @param[in] parent_frame_id ROS frame ID of the camera (e.g., "cam0").
 * @param[in] child_frame_id ROS frame ID of the robot base (e.g., "fr3_link0").
 */
void publish_transforms(
    MultiCameras& cameras, 
    Eigen::MatrixXf& transform,
    tf2_ros::StaticTransformBroadcaster& broadcaster,
    std::string& parent_frame_id,
    std::string& child_frame_id
);

/**
 * @brief Publishes transforms and visualization markers for both local and triangulated ArUco patterns.
 *
 * This function launches a loop that continuously publishes:
 * - Static transforms using the computed calibration.
 * - ROS topics for marker visualization:
 *   - `"triangulated"` for markers reconstructed from cameras
 *   - `"local"` for ground truth pattern points
 *
 * @param[in] nh ROS node handle.
 * @param[in] cameras Reference to the multi-camera configuration.
 * @param[in] transform The estimated camera-to-robot transformation matrix.
 * @param[in] parent_frame_id Frame ID for the camera system.
 * @param[in] child_frame_id Frame ID for the robot base.
 * @param[in] local_points Ground truth marker points in robot frame.
 * @param[in] triangulated_points Marker points reconstructed via triangulation.
 * @param[in] ids Marker IDs corresponding to the above points.
 */
void publish_result(
    ros::NodeHandle& nh, MultiCameras& cameras, 
    Eigen::MatrixXf& transform,
    std::string& parent_frame_id,
    std::string& child_frame_id,
    std::vector<Eigen::VectorXf>& local_points,
    std::vector<Eigen::VectorXf>& triangulated_points, 
    std::vector<int>& ids
);

/**
 * Loads result for .json file
 *
 * @param[in] path Path to result file
 * @param[out] ids Corresponding Ids of the aruco definition
 * @param[out] triangulated_points Triangulated Points of the aruco markers Nx3
 * @param[out] transformation Transformation Matrix
 * \return true if result could get loaded, false otherwise
 */
bool load_single_result(
    std::string path,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transformation
);

} // namespace flir_icp_calib