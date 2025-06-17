#include "interface.hpp"

namespace flir_icp_calib{

TransformInterfaceAruco::TransformInterfaceAruco(ros::NodeHandle &nh, const std::string &cfg_name)
{
    this->nh = nh;
    this->type = "static_transform_publisher";
    load_transform_config(cfg_name, this->cfg);
    if (!load_calibration(this->cfg.calibration_file, this->cameras)){
        std::string msg = "Faild to load cameras";
        spdlog::error(msg);
        throw std::runtime_error(msg);
    };
    this->pub_det_aruco=this->nh.advertise<keiko_msgs::ArucoMarkers3d>(
        std::string("aruco_corrdinates_")+this->cfg.parent_frame_id, 2);
    this->pub_det_rviz=this->nh.advertise<visualization_msgs::MarkerArray>(
        "det_aruco_markers", 2);

};
bool TransformInterfaceAruco::get_images(
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> &frame
){
    flirmulticamera::CameraSettings settings;
    spdlog::info("Loading camera settings from: {}", this->cfg.camera_settings_file);
    load_camera_settings(this->cfg.camera_settings_file, settings);
    flirmulticamera::FlirCameraHandler fcamhandler(
        settings
    );
    if (!fcamhandler.Configure())
    {
        spdlog::error("Could not configure FlirCameraHandler...");
        return false;
    }
    fcamhandler.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    size_t failed_iterations = 0;
    constexpr size_t max_faild_iterations = 100;
    std::array<flirmulticamera::Frame, flirmulticamera::GLOBAL_CONST_NCAMS> imgs;
    while(true)
    {
        if (fcamhandler.Get(imgs))
        {
            spdlog::info("Got images");
            break;
        }
        else
        {
            failed_iterations++;
            if (failed_iterations > max_faild_iterations)
            {
                spdlog::error("Max number of failed iterations reached: {}", max_faild_iterations);
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fcamhandler.Stop();
    bool success = true;
    for (uint8_t cidx = 0; cidx < imgs.size(); cidx++)
    {
        flirmulticamera::Frame &img = imgs.at(cidx);
        if (img.frameData == nullptr)
        {
            spdlog::error("Image is null...");
            success = false;
            continue;
        }
        {
            if (img.frameData->IsIncomplete())
            {
                spdlog::error("Image incomplete with image status {}...", img.frameData->GetImageStatus());
                success = false;
                continue;
            }
            else
            {   
                cv::Mat tmp(img.frameData->GetHeight(), img.frameData->GetWidth(), CV_8UC3, img.frameData->GetData());
                frame.at(cidx) = tmp.clone();
                cv::cvtColor(frame.at(cidx), frame.at(cidx), cv::COLOR_RGB2BGR);
            }

        }
    }
    return success;
};

bool TransformInterfaceAruco::start()
{
    std::string frame_dir = std::string(CONFIG_DIR)+"/../test/data";
    // get and save images
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    if (this->cfg.online){
        this->get_images(frame);
    }
    else{
        std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> fnames = cpp_utils::get_filenames<flirmulticamera::GLOBAL_CONST_NCAMS>(
            this->cfg.load_path_offline, ".jpg"
        );
        for (uint8_t i =0; i<flirmulticamera::GLOBAL_CONST_NCAMS; i++){
            std::string inputImage =  this->cfg.load_path_offline+fnames.at(i)+".jpg";
            frame.at(i) = cv::imread(inputImage);
            if (frame.at(i).empty()){
                const std::string msg = "Unable to read image at path: " + inputImage;
                spdlog::error(msg);
                throw std::runtime_error(msg);
            }
        }
    }
    std::vector<int> ids_detected, ids_pattern; 
    std::array<std::vector<std::vector<cv::Point2f>>, flirmulticamera::GLOBAL_CONST_NCAMS> detected_markers;
    // image, marker, corner
    scan_pattern(frame, this->cfg, ids_detected, detected_markers);
    if ((int) ids_detected.size() < this->cfg.min_markers){
        std::string msg = "Not enough markers: Detected "+ std::to_string(ids_detected.size()) + 
            ", Needed "+std::to_string(this->cfg.min_markers);
        spdlog::error(msg);
        throw std::runtime_error(msg);
    }

    std::string savepath = std::string(CONFIG_DIR)+"/../log";
    if (this->cfg.log){
        std::string timestamp = cpp_utils::get_timestamp();
        savepath +="/"+timestamp;
        if (!std::filesystem::create_directory(savepath)){
            std::string msg = "Could not create directory: " + savepath;
            spdlog::error(msg);
            throw std::runtime_error(msg);
        };
        // save images
        // for (std::size_t img_id = 0; img_id<frame.size(); img_id++){
        //     cv::imwrite(savepath+"/"+std::to_string(img_id)+".jpg", frame.at(img_id));
        // }
        // save detected markers
        for (int img_id = 0; img_id<frame.size(); img_id++){
            cv::Mat img_cp = frame.at(img_id).clone();
            cv::aruco::drawDetectedMarkers(img_cp, detected_markers.at(img_id), ids_detected);
            cv::imwrite(savepath+"/"+std::to_string(img_id)+"_det.jpg", img_cp);
        }
    }
    //load local 3D pattern
    std::vector<Eigen::VectorXf> local_points;
    arucodata_from_json(this->cfg.local_pattern_file, ids_pattern, local_points);
    spdlog::info("Loaded arucodata");
    spdlog::info("N Markers Detected: {}", ids_detected.size());
    spdlog::info("N Markers Pattern: {}", ids_pattern.size());

    // intersect pattern & detected
    // fill camids, fill detected markers
    std::vector<std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS>> detected_points;
    std::vector<std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS>> CamIDs;
    std::vector<int> ids_intersection;
    std::vector<Eigen::VectorXf> local_points_intersection;
    for (int pidx=0; pidx<ids_pattern.size(); pidx++){ // point in local pattern
        for (int midx=0; midx<ids_detected.size(); midx++){ // detected marker
            if (ids_detected.at(midx) == ids_pattern.at(pidx)){
                int id_i = ids_pattern.at(pidx);
                ids_intersection.push_back(id_i);
                for (int cid=0; cid<4; cid++){ // corners
                    Eigen::VectorXf _local_point(3);
                    _local_point = local_points.at(pidx*4+cid);
                    local_points_intersection.push_back(_local_point);
                    std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> img_points; 
                    for (int img_id=0; img_id<frame.size(); img_id++){
                        img_points.at(img_id) = detected_markers.at(img_id).at(midx).at(cid);
                    }
                    detected_points.push_back(img_points);
                    // Points have been detected by all cameras (scan pattern)
                    std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS> cids;
                    for (uint8_t i = 0; i < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); i++)
                    {
                        cids.at(i) = i;
                    }
                    CamIDs.push_back(cids);
                }
            }
        }
    }
    spdlog::info("N_ids intersection: {}", ids_intersection.size());

    triangulate_multi_cams(
        this->cameras, this->cfg.max_reprojection_error_images, 
        detected_points, CamIDs, this->triangulated_points);


    iterative_closest_points(local_points_intersection, this->triangulated_points, this->transform);
    std::cout<<transform.matrix()<<std::endl;
    
    if (!validate_reprojection_after_icp(
        this->transform, 
        local_points_intersection, 
        this->cameras, frame, savepath,
        detected_points, CamIDs, this->cfg.max_reprojection_error_robot, this->cfg.log
    )){
        throw std::runtime_error("Failed to validate result");
        return false;
    }
    // END gen transform

    if (this->cfg.log){
        save_result(
            savepath,
            this->cfg.calibration_file, this->cfg.local_pattern_file,
            ids_intersection,
            this->triangulated_points, this->transform
        );
    }

    publish_transforms(
        this->cameras, this->transform, this->broadcaster, 
        this->cfg.parent_frame_id, this->cfg.child_frame_id
    );

    std_msgs::ColorRGBA color;
    color.a=1;
    color.r=1;
    color.g=1;

    gen_aruco_msgs(
        triangulated_points, ids_intersection,
        this->cfg.parent_frame_id, color,
        this->det_arucomarkers3d, this->det_rviz
    );

     this->ThreadHandle.reset(new std::thread(&TransformInterfaceAruco::ThreadFunction, this));

    return true;
};

void TransformInterfaceAruco::ThreadFunction()
{
    uint16_t count = 0;
    while (!this->ShouldClose && ros::ok()){
        count ++;
        if (count == 100){
            this->pub_det_aruco.publish(this->det_arucomarkers3d);
            this->pub_det_rviz.publish(this->det_rviz);
            count == 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

void TransformInterfaceAruco::Terminate(){
    this->ShouldClose = true;
    this->ThreadHandle->join();
}

}
