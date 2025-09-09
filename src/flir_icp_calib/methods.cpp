#include "methods.hpp"

namespace flir_icp_calib{

bool scan_pattern(
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS>& frame, config_transform& cfg_transform, std::vector<int>& ids, 
    std::array<std::vector<std::vector<cv::Point2f>>, flirmulticamera::GLOBAL_CONST_NCAMS>& detected_markers // img, marker, corners
)
{
    for (int img_id = 0; img_id<frame.size(); img_id++){
        detected_markers.at(img_id).clear();
    }
    // aruco
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cfg_transform.aruco_dictionary);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    // buffers for detections
    std::vector<std::vector<int>> _markerIds;
    std::vector<std::vector<std::vector<cv::Point2f>> > _markerCorners;
    std::vector<std::vector<std::vector<cv::Point2f>> > _rejectedCandidates;
    
    for (int img_id = 0; img_id<frame.size(); img_id++){
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        detector.detectMarkers(frame.at(img_id), markerCorners, markerIds, rejectedCandidates);
        _markerIds.push_back(markerIds);
        _markerCorners.push_back(markerCorners);
        _rejectedCandidates.push_back(rejectedCandidates);
    }

    // intersection
    int max_ids = cfg_transform.h*cfg_transform.w;
    for (int id =0; id<max_ids; id++){
        std::vector<std::vector<cv::Point2f>> _intersection_corners; 
        for (int img_id = 0; img_id<frame.size(); img_id++){ //per image
            for (int idx = 0; idx<_markerIds.at(img_id).size(); idx++){
                if (_markerIds.at(img_id).at(idx) == id){
                    _intersection_corners.push_back(_markerCorners.at(img_id).at(idx));
                }
            }
        }
        if(_intersection_corners.size() == frame.size() ){
            ids.push_back(id);
            for (int img_id = 0; img_id<frame.size(); img_id++){
                detected_markers.at(img_id).push_back(_intersection_corners.at(img_id)); 
            }
        }
    }
    return true;
}

std::vector<float> mean_reprojection_errors(
    MultiCameras& cameras, std::vector<Eigen::VectorXf>& points_3d, 
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>>& detected_markers, 
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>>& CamIDs
)
{
    std::vector<float> cerrors(flirmulticamera::GLOBAL_CONST_NCAMS, 0);
    float counter = 0;
    for(int point_idx = 0; point_idx<detected_markers.size(); point_idx++)
    {
        assert(CamIDs.at(point_idx).size() == detected_markers.at(point_idx).size());
        Eigen::VectorXf point_3d{4};
        point_3d << points_3d.at(point_idx)(0), points_3d.at(point_idx)(1), points_3d.at(point_idx)(2), 1.;
        int camera_id;
        for (int idx = 0; idx<CamIDs.at(point_idx).size(); idx++)
        {
            camera_id = CamIDs.at(point_idx).at(idx);
            Eigen::Vector3f reprojected = cameras.Cam.at(camera_id).P * point_3d;
            reprojected /= reprojected(2);
            Eigen::Vector2f projected_e;
            projected_e(0) = reprojected(0) - detected_markers.at(point_idx).at(idx).x;
            projected_e(1) = reprojected(1) - detected_markers.at(point_idx).at(idx).y;
            cerrors.at(camera_id) += projected_e.norm();
        }
        counter+=1.;
    }
    for (int i =0; i<flirmulticamera::GLOBAL_CONST_NCAMS; i++){
        cerrors.at(i)/=counter;
    }

    return cerrors;
}

void Calc3DCentre(
    std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> Points, 
    std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS> CamIDs, 
    MultiCameras Cameras, Eigen::VectorXf& Centre3D
){
    // assert(CamIDs.size() == Points.size());
    uint8_t numOfPoints = (uint8_t)Points.size();

    // Form H * Centre3D = 0

    Eigen::MatrixXf H(numOfPoints * 2, 4);

    // range flirmulticamera::GLOBAL_CONST_NCAMS
    for (Eigen::Index i = 0; i < numOfPoints; i++)
    {
        // H ?
        // CamIDs ?
        H(2 * i, 0) = Cameras.Cam[CamIDs[i]].P(0, 0) - Cameras.Cam[CamIDs[i]].P(2, 0) * Points[i].x;
        H(2 * i, 1) = Cameras.Cam[CamIDs[i]].P(0, 1) - Cameras.Cam[CamIDs[i]].P(2, 1) * Points[i].x;
        H(2 * i, 2) = Cameras.Cam[CamIDs[i]].P(0, 2) - Cameras.Cam[CamIDs[i]].P(2, 2) * Points[i].x;
        H(2 * i, 3) = Cameras.Cam[CamIDs[i]].P(0, 3) - Cameras.Cam[CamIDs[i]].P(2, 3) * Points[i].x;

        H(2 * i + 1, 0) = Cameras.Cam[CamIDs[i]].P(1, 0) - Cameras.Cam[CamIDs[i]].P(2, 0) * Points[i].y;
        H(2 * i + 1, 1) = Cameras.Cam[CamIDs[i]].P(1, 1) - Cameras.Cam[CamIDs[i]].P(2, 1) * Points[i].y;
        H(2 * i + 1, 2) = Cameras.Cam[CamIDs[i]].P(1, 2) - Cameras.Cam[CamIDs[i]].P(2, 2) * Points[i].y;
        H(2 * i + 1, 3) = Cameras.Cam[CamIDs[i]].P(1, 3) - Cameras.Cam[CamIDs[i]].P(2, 3) * Points[i].y;
    }
    // Calculate Centre3D using SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinV);
    auto V = svd.matrixV();
    Centre3D = V.col(V.cols() - 1);
    Centre3D /= Centre3D(3);
}

std::vector<Eigen::VectorXf> Calc3DCentres(
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>> Points, 
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>> CamIDs, 
    MultiCameras Cameras
){
    std::vector<Eigen::VectorXf> Points3D;
    Points3D.reserve(100);
    for (size_t point_inx = 0; point_inx < Points.size(); point_inx++)
    {
        Eigen::VectorXf point3D(3);
        Calc3DCentre(Points[point_inx], CamIDs[point_inx], Cameras, point3D);
        Points3D.push_back(point3D);
    }
    return Points3D;
}

bool triangulate_multi_cams(
    MultiCameras& cameras, float max_reprojection_error, 
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>>& detected_markers, 
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>>& CamIDs, 
    std::vector<Eigen::VectorXf>& points_3d
){
    points_3d.clear();
    points_3d = Calc3DCentres(detected_markers, CamIDs, cameras);
    bool success = true;
    std::vector<float> cerrors = mean_reprojection_errors(cameras, points_3d, detected_markers, CamIDs);
    spdlog::info("Reprojection Errors:");
    for (std::size_t i = 0; i<flirmulticamera::GLOBAL_CONST_NCAMS; i++)
    {
        std::string msg = "{}:\t{}";
        if (cerrors.at(i) > max_reprojection_error){
            msg += ">= treshold! ({})";
            spdlog::error(
                msg, 
                cameras.Cam.at(i).SN,
                cerrors.at(i),
                max_reprojection_error
            );
            success = false;
        }
        else{
            spdlog::info(
                msg, 
                cameras.Cam.at(i).SN,
                cerrors.at(i)
            );
        }
    }
    return success;
}

Eigen::Vector3f get_normal(Eigen::MatrixXf points){

    // Compute the centroid of the points
    Eigen::Vector3f centroid = points.colwise().mean();

    // Compute the covariance matrix
    Eigen::Matrix3f covariance = (points.transpose() * points) / double(points.rows());

    // Perform PCA
    Eigen::EigenSolver<Eigen::MatrixXf> solver(covariance);
    Eigen::Vector3f eigenvalues = solver.eigenvalues().real();
    Eigen::MatrixXf eigenvectors = solver.eigenvectors().real();

    // The normal vector of the plane is the eigenvector corresponding to the smallest eigenvalue
    Eigen::Vector3f normal = eigenvectors.col(0);

    Eigen::Vector3f normal_normalized = normal.transpose();
    normal_normalized.normalize();

    std::cout << "Normal vector: " << normal_normalized << std::endl;
    return normal_normalized;
}

void shift_z(Eigen::MatrixXf& mat_source, Eigen::MatrixXf& mat_target, const uint32_t n_points)
{
    // get normals
    // Eigen::Vector3f normal_target = get_normal(mat_target);
    Eigen::Vector3f normal_target{0,0,0};
    Eigen::Vector3f normal_source{0,0,0};
    Eigen::Vector3f zeros{0,0,0};

    int max_points = mat_source.cols();
    assert(n_points<max_points);

    Eigen::MatrixXf shift_source(max_points, 3);
    Eigen::MatrixXf shift_target(max_points, 3);

    for(uint i=0; i<max_points; i++){
        if (i<n_points){
            mat_source.row(i) = normal_source;
            mat_target.row(i) = normal_target;
        }
        else{
            mat_source.row(i) = zeros;
            mat_target.row(i) = zeros;
        }
    }
}

bool iterative_closest_points(
    std::vector<Eigen::VectorXf> v_source, std::vector<Eigen::VectorXf> v_target, 
    Eigen::MatrixXf& transform
){
    assert(v_source.size()==v_target.size());
    transform = Eigen::MatrixXf(4,4);

    Eigen::Vector3f mean_source{0,0,0};
    Eigen::Vector3f mean_target{0,0,0};

    int NumOfPoints = v_source.size();
    Eigen::MatrixXf mat_source(NumOfPoints, 3);
    Eigen::MatrixXf mat_target(NumOfPoints, 3);

    // convert to non-homogeneous format and compute mean
    for (int i = 0; i<NumOfPoints; i++)
    {
        // convert to 3f
        Eigen::Vector3f p_source_tmp;
        Eigen::Vector3f p_target_tmp;
        p_source_tmp << v_source.at(i).head<3>();
        p_target_tmp << v_target.at(i).head<3>();
        mat_source.row(i) = p_source_tmp;
        mat_target.row(i) = p_target_tmp;
        mean_source += p_source_tmp;
        mean_target += p_target_tmp;
    }
    mean_source /= NumOfPoints;
    mean_target /= NumOfPoints;
    // substract means
    mat_source = mat_source.rowwise()-mean_source.transpose();
    mat_target = mat_target.rowwise()-mean_target.transpose();

    // compute rotation
    Eigen::MatrixXf S = mat_source.transpose() * mat_target;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXf R_ = svd.matrixV() * svd.matrixU().transpose();
    Eigen::MatrixXf W = Eigen::MatrixXf::Identity(3, 3);
    W(2, 2) = R_.determinant();
    Eigen::MatrixXf R = svd.matrixV() * W * svd.matrixU().transpose();
    // compute translation
    Eigen::Vector3f t = mean_target - R * mean_source;
    // convert to homogeneous transformation matrix
    transform.fill(0);
    transform.block<3, 3>(0, 0) = R;
    transform(0, 3) = t(0);
    transform(1, 3) = t(1);
    transform(2, 3) = t(2);
    transform(3, 3) = 1.;

    transform = transform.inverse();

    return true;
}


bool validate_reprojection_after_icp(
    Eigen::MatrixXf &transform, 
    std::vector<Eigen::VectorXf> &local_points_intersection, 
    MultiCameras &cameras,
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> &frame, 
    std::string &save_dir,
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>> &detected_points,
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>> &CamIDs,
    float &max_reprojection_error_robot, bool &log
){
    // reprojection check of triangulated points 
    Eigen::MatrixXf transform_inv = transform.inverse();
    std::vector<Eigen::VectorXf> points_projected;
    for (auto local : local_points_intersection){
        Eigen::VectorXf _local(4);
        _local << local[0], local[1], local[2], 1.0f;  
        _local = transform_inv*_local;
        Eigen::VectorXf projected(3);
        projected << _local[0]/_local[3], _local[1]/_local[3], _local[2]/_local[3];
        points_projected.push_back(projected);
    }
    if (log){
        // draw markers to validate reprojection errors
        for(int point_idx = 0; point_idx<detected_points.size(); point_idx++)
        {
            assert(CamIDs.at(point_idx).size() == detected_points.at(point_idx).size());
            Eigen::VectorXf point_3d{4};
            point_3d << points_projected.at(point_idx)(0), 
            points_projected.at(point_idx)(1),
            points_projected.at(point_idx)(2),
            1.;
            int camera_id;
            for (int cidx = 0; cidx<CamIDs.at(point_idx).size(); cidx++)
            {
                camera_id = CamIDs.at(point_idx).at(cidx);
                Eigen::Vector3f reprojected = cameras.Cam.at(camera_id).P * point_3d;
                reprojected /= reprojected(2);
                cv::Point2f rep;
                rep.x = reprojected(0);
                rep.y = reprojected(1);
                cv::Scalar color_rep( 255, 0, 0 );
                cv::Scalar color_det( 0, 255, 0 );
                
                cv::drawMarker(frame.at(cidx), rep, color_rep);
                cv::drawMarker(frame.at(cidx), detected_points.at(point_idx).at(cidx), color_det);
            }
        }
        for (int img_id = 0; img_id<frame.size(); img_id++){
            cv::imwrite(save_dir+"/"+std::to_string(img_id)+"_rep.jpg", frame.at(img_id));
        }
    }

    spdlog::info("Reprojection Errors of Back-projected pattern:");

    std::vector<float> errors2 = mean_reprojection_errors(cameras, points_projected, detected_points, CamIDs);
    for (int i = 0; i<frame.size(); i++){
        spdlog::info("{}\t{}", cameras.Cam.at(i).SN, errors2.at(i));
    }
    bool success = true;
    for (int i = 0; i<frame.size(); i++){
        if (errors2.at(i) > max_reprojection_error_robot){
            spdlog::error(
                "Reprojection of local 3D to {}: {} exceeds threshold: {}",
                cameras.Cam.at(i).SN,
                errors2.at(i),
                max_reprojection_error_robot
            );
            success = false;
        }
    }
    if (!success){
        throw std::runtime_error("Back Projected Local Pattern exceeds reprojection error threshold!"
        );
        return false;
    }
    return true;
}

} // namespace flir_icp_calib
