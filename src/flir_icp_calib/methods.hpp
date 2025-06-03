#pragma once

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include "config_parser.hpp"
#include "camera_config.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace flir_icp_calib{
/**
 * Scans Images aruco markers, defined by config
 *
 * @param [in] frame Images to scan
 * @param [in] config_transform transform config
 * @param [out] ids ids detected on all images (intersection)
 * @param [out] detected_markers Points of the detected markers, sorted by given ids and corners, Npoints(Nmakersx4)xNcamsx2
 * @param [out] CamIDs ids of cameras points were detected with shape Npoints x Ndetectedcams
 * \return true if n_markers_detected exceeds min markers defined by cfg
 */
bool scan_pattern(
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS>& frame, config_transform& cfg_transform, std::vector<int>& ids, 
    std::array<std::vector<std::vector<cv::Point2f>>, flirmulticamera::GLOBAL_CONST_NCAMS>& detected_markers
);

/**
 * Computes the reprojection errors of triangulated points per camera
 *
 * @param [in] cameras N Camera objects containing the projection matrices 
 * @param [in] points_3d Triangulated points of shape Nx3
 * @param [in] detected_markers Corresponding 2d points of shape NxNcamsx2 
 * @param [out] CamIDs ids of cameras points were detected with shape Npoints x Ndetectedcams
 * \return Camera-wise mean reprojection error
 */
std::vector<float> mean_reprojection_errors(
    MultiCameras& cameras, std::vector<Eigen::VectorXf>& points_3d, 
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>>& detected_markers,
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>>& CamIDs
);

/**
 * Triangulates N points detcted by multiple cameras and checks the reprojection error
 *
 * @param [in] Points Points to triangulate N_detected_cams x 2
 * @param [in] CamIDs Ids for all cameras of shape N_detected_cams
 * @param [in] Cameras Cameras object containing projections for all cameras
 * @param [out] Centre3D Triangulated center in 3d space. Center is 4 dimensional as it is homogeneous!
 * \return Void
 */
void Calc3DCentre(
    std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> Points, 
    std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS> CamIDs, MultiCameras Cameras, 
    Eigen::VectorXf& Centre3D
);

/**
 * Triangulates N points detected by Ndetected cameras
 *
 * @param [in] Points Points to triangulate with shape Npoints x Ndetected x 2
 * @param [in] CamIDs ids of cameras points were detected with shape Npoints x Ndetectedcams
 * @param [in] Cameras Cameras object containing projections for all cameras
 * \return Triangulated homogeneous points with shape Nx4  
 */
std::vector<Eigen::VectorXf> Calc3DCentres(
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>> Points, 
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>> CamIDs, 
    MultiCameras Cameras
);

/**
 * Triangulates N points detcted by multiple cameras and checks the reprojection error
 *
 * @param [in] cameras N Camera objects containing the projection matrices 
 * @param [in] max_reprojection_error Maximum mean reprojection error of 3D point to image 
 * @param [in] detected_markers Set of dected points, Npoints x Ndetectedcams x 2
 * @param [out] CamIDs ids of cameras points were detected with shape Npoints x Ndetectedcams
 * @param [out] points_3d Triangulated 3d points of shape Npoints x 3
 * \return true if reprojection error < max_reprojection_error for all cameras, false otherwise
 */
bool triangulate_multi_cams(
    MultiCameras& cameras, float max_reprojection_error, 
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>>& detected_markers, 
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>>& CamIDs, 
    std::vector<Eigen::VectorXf>& points_3d
);

/**
 * Implementation of Iterative Closest Points Algorithm by
 * https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
 *
 * @param [in] v_source Point cloud of the source frame
 * @param [in] v_target Point cloud of the target frame
 * @param [out] transform Computed homogeneous transformation matrix of shape 4x4

 * \return true if all intermediated operations return true, false otherwise
 */
bool iterative_closest_points(
    std::vector<Eigen::VectorXf> v_source, std::vector<Eigen::VectorXf> v_target, Eigen::MatrixXf& transform
);

/**
 * Checks how well the local 3D pattern matches the detected markers 
 * via re-projecting them into the original images
 *
 * @param [in] transform Computed homogeneous transformation matrix of shape 4x4
 * @param [in] local_points_intersection 3D points of local pattern
 * @param [in] cameras cameras object
 * @param [in] frame images
 * @param [in] save_dir save directory for logging the images
 * @param [in] detected_points 2D detected points
 * @param [in] CamIDs Camera id for each detection
 * \return true if reprojection error < max_reprojection_error for all cameras, false otherwise
 */
bool validate_reprojection_after_icp(
    Eigen::MatrixXf &transform, 
    std::vector<Eigen::VectorXf> &local_points_intersection, 
    MultiCameras &cameras,
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> &frame, 
    std::string &save_dir,
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>> &detected_points,
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>> &CamIDs,
    float &max_reprojection_error_robot, bool &log
);

} // namespace flir_icp_calib