#include "dataIO.hpp"

namespace flir_icp_calib{


bool arucodata_from_json(
    std::string& calib_path,
    std::vector<int>& ids, std::vector<Eigen::VectorXf>& points_arucos
){
    Document document;
    if (!cpp_utils::load_json_with_schema(
        calib_path, std::string(CONFIG_DIR)+"/../data/local_calibration.schema.json",
        65536, document
    )){return false;}

    points_arucos.clear();
    ids.clear();
    for (int id = 0; id<80; id++){
        std::string _id = std::to_string(id);
        if (document.HasMember(_id.c_str())){
            ids.push_back(id);
            auto pjsonarray = document[_id.c_str()].GetArray();

            for (int c = 0; c<4; c++){
                Eigen::VectorXf point(3);
                point << 
                    pjsonarray[c][0].GetFloat(), 
                    pjsonarray[c][1].GetFloat(), 
                    pjsonarray[c][2].GetFloat();
                points_arucos.push_back(point);
            }
        }
    }

    return true;
}


bool result_to_json(
    std::string path,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transformation
)
{
    Document doc;
    doc.SetObject();

    // check if empty
    assert(ids.size() != 0);
    assert(triangulated_points.size() != 0);

    rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
    std::string ts = cpp_utils::get_timestamp();  // ts must stay in scope
    rapidjson::Value timestamp_val;
    timestamp_val.SetString(ts.c_str(), static_cast<rapidjson::SizeType>(ts.length()), allocator);
    doc.AddMember("timestamp", timestamp_val, allocator);

    Value ID_ArrayValue(kArrayType);
    for (auto id :ids) {
        ID_ArrayValue.PushBack(id, doc.GetAllocator());
    }
    doc.AddMember("ids", ID_ArrayValue, doc.GetAllocator());

    Value TP_ArrayValue(kArrayType);
    for (auto tp : triangulated_points) {
        for(int i=0; i<3; i++){
            TP_ArrayValue.PushBack(tp(i), doc.GetAllocator());
        }
    }
    doc.AddMember("triangulated_points", TP_ArrayValue, doc.GetAllocator());

    Value TF_ArrayValue(kArrayType);
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++){
            TF_ArrayValue.PushBack(transformation(i, j), doc.GetAllocator());
        }
    }
    doc.AddMember("transform", TF_ArrayValue, doc.GetAllocator());

    // save json
    std::string filename = path+"/result.json";

    spdlog::info("Saving {}", filename);
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::string error_msg = "Unable to open file: "+filename;
        spdlog::error(error_msg);
        throw std::runtime_error(error_msg);
        return false;
    }
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.SetIndent(' ', 4);
    doc.Accept(writer);
    ofs << buffer.GetString();
    ofs.close();


    return true;
}

bool save_result(
    std::string &savepath,
    std::string &calibration_file,
    std::string &local_pattern_file,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transform
)
{
    std::filesystem::copy_file(
        std::string(CONFIG_DIR)+"/TransformsSettings.json",
        savepath+"/TransformsSettings.json"
    );
    std::filesystem::copy_file(
        calibration_file,
        savepath+"/Calibration.json"
    );
    std::filesystem::copy_file(
        local_pattern_file,
        savepath+"/local_pattern.json"
    );
    if (
        !result_to_json(
            savepath, ids, 
            triangulated_points, transform
        )
    )
    {
        return false;
    };
    return true;
}

void publish_transforms(
    MultiCameras& cameras, 
    Eigen::MatrixXf& transform,
    tf2_ros::StaticTransformBroadcaster& broadcaster,
    std::string& parent_frame_id,
    std::string& child_frame_id

)
{
    // set multi-camera-transforms
    for (std::size_t cam_id = 1; cam_id <GLOBAL_CONST_NCAMS; cam_id++){
        auto msg = matrix2msg(cameras.Cam.at(cam_id).M, parent_frame_id, "cam"+std::to_string(cam_id), true);
        broadcaster.sendTransform(msg);
    }

    geometry_msgs::TransformStamped msg = matrix2msg(
        transform, parent_frame_id, child_frame_id, true
    );
    broadcaster.sendTransform(msg);

};

void publish_result(
    ros::NodeHandle& nh, MultiCameras& cameras, 
    Eigen::MatrixXf& transform,
    std::string& parent_frame_id,
    std::string& child_frame_id,
    std::vector<Eigen::VectorXf>& local_points,
    std::vector<Eigen::VectorXf>& triangulated_points, 
    std::vector<int>& ids
)
{
    tf2_ros::StaticTransformBroadcaster broadcaster;
    publish_transforms(
        cameras, transform, broadcaster, parent_frame_id, child_frame_id
    );


    ros::Publisher pub_triangulated = nh.advertise<visualization_msgs::MarkerArray>("triangulated", 10);
    ros::Publisher pub_local = nh.advertise<visualization_msgs::MarkerArray>("local", 10);

    visualization_msgs::MarkerArray triangulated_markers;
    visualization_msgs::MarkerArray local_markers;
    std_msgs::ColorRGBA color;
    color.r = 1.;
    color.a = 0.5;

    add_markers(triangulated_points, ids, color, parent_frame_id, triangulated_markers);
    color.g = 1.;
    add_markers(local_points, ids, color, child_frame_id, local_markers);

    // Spin and publish the Markers
    while (ros::ok())
    {
        pub_triangulated.publish(triangulated_markers);
        pub_local.publish(local_markers);
        ros::spinOnce();
        ros::Rate(1).sleep();
    }

}

bool load_single_result(
    std::string path,
    std::vector<int>& ids, 
    std::vector<Eigen::VectorXf>& triangulated_points,
    Eigen::MatrixXf& transformation
)
{
    ids.clear();
    triangulated_points.clear();
    
    Document document;
    if (!cpp_utils::read_json_document(path, 65536, document)){
        return false;
    }
    // convert ids
    assert(document.HasMember("ids"));
    assert(document["ids"].IsArray());
    auto idsjsonarray = document["ids"].GetArray();
    for (int i = 0; i < idsjsonarray.Size(); ++i)
    {
        ids.push_back(idsjsonarray[i].GetInt());
    }

    // convert triangulated_points
    assert(document.HasMember("triangulated_points"));
    assert(document["triangulated_points"].IsArray());
    auto triangulated_pointsarray = document["triangulated_points"].GetArray();
    int n_points_t = triangulated_pointsarray.Size()/3;
    // assert(n_points_t == n_points);
    for (int i = 0; i < n_points_t; ++i)
    {
        Eigen::VectorXf point_tmp{3};
        point_tmp(0) = triangulated_pointsarray[i*3].GetFloat();
        point_tmp(1) = triangulated_pointsarray[i*3+1].GetFloat();
        point_tmp(2) = triangulated_pointsarray[i*3+2].GetFloat();
        triangulated_points.push_back(point_tmp);
    }

    // convert transform
    assert(document.HasMember("transform"));
    assert(document["transform"].IsArray());
    auto transformarray = document["transform"].GetArray();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            transformation(i, j) = transformarray[i*4+j].GetFloat();
        }
    }

    return true;
}


} //namespace flir_icp_calib