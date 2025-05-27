#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "keiko_msgs/ArucoMarkers3d.h"

namespace flir_icp_calib{

/**
 * Converts 4x4 Eigen matrix inot transform stamped message
 *
 * @param [in] transformation_matrix Homogeneous transformation matrix
 * @param [in] parent_frame Parent frame identifier
 * @param [in] child_frame Child frame identifier
 * \return Converted transform as message
 */
geometry_msgs::TransformStamped matrix2msg(
    Eigen::MatrixXf transformation_matrix, const std::string parent_frame, const std::string child_frame, 
    const bool inverse
);


/**
 * Appends aruco corners points to marker array of text-markers.
 * The markers are denoted acording to the corner ids and their.
 *
 * @param [in] points Points of the markers
 * @param [in] ids Ids of the markers
 * @param [in] color Marker-color
 * @param [in] frame_id Id of the markers to be publihed in
 * @param [in] marker_array Marker Array the points are appended to
 * \return Void
 */
void add_markers(
    std::vector<Eigen::VectorXf>& points, std::vector<int>& ids, std_msgs::ColorRGBA& color, 
    std::string frame_id, visualization_msgs::MarkerArray& marker_array
);

/**
 * Converts result into two MarkerArrays for checkup
 * local (definition/robot) is red, triangulated (camera) are yellow
 * 
 * @param [in] names Identifiers of the aruco descriptions
 * @param [in] res_points_arucos  Local Aruco corners for each definition
 * @param [in] res_ids Aruco Ids for each definition
 * @param [in] res_triangulated_points Triangulated Points for each definition and cam
 * @param [out] triangulated_markers Triangulated MarkerArray
 * @param [out] local_markers Local (Definition) MarkerArray
 * \return void
 */

void marker_visualization(
    std::vector<std::string>& names, 
    std::vector<std::vector<Eigen::VectorXf>>& res_points_arucos, 
    std::vector<std::vector<int>>& res_ids, 
    std::vector<std::vector<Eigen::VectorXf>>& res_triangulated_points,
    visualization_msgs::MarkerArray& triangulated_markers,
    visualization_msgs::MarkerArray& local_markers,
    std::string &frame_id
);

/**
 * Generates aruco messages (position and rviz markers) i.e. for
 * local (definition/robot) red, triangulated (camera) are yellow
 * 
 * @param [in] points list of 3d points
 * @param [in] ids  marker ids
 * @param [in] frame_id frame_id i.e. 'fr3_link0' or 'cam0'
 * @param [in] color red for local/robot, yellow for cam0
 * @param [out] arucomarkers3d positions msg of aruco markers
 * @param [out] marker_array rviz markers
 * \return void
 */
void gen_aruco_msgs(
    std::vector<Eigen::VectorXf>& points, std::vector<int>& ids,
    std::string frame_id, std_msgs::ColorRGBA color,
    keiko_msgs::ArucoMarkers3d &arucomarkers3d, 
    visualization_msgs::MarkerArray& marker_array
);

}
