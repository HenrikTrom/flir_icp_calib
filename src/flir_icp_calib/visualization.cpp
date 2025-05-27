#include "visualization.hpp"

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
    Eigen::MatrixXf transformation_matrix, const std::string parent_frame, 
    const std::string child_frame, const bool inverse
){
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = parent_frame; // Change to your parent frame ID
    transform_stamped.child_frame_id = child_frame;   // Change to your child frame ID

    if (inverse){
        transformation_matrix = transformation_matrix.inverse();
    }

    // Fill in the translation
    transform_stamped.transform.translation.x = transformation_matrix(0, 3);
    transform_stamped.transform.translation.y = transformation_matrix(1, 3);
    transform_stamped.transform.translation.z = transformation_matrix(2, 3);

    // Convert the rotation matrix to quaternion
    Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation_matrix);
    // quaternion.normalize();
    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.y();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();
    return transform_stamped;
}

void add_markers(
    std::vector<Eigen::VectorXf>& points, std::vector<int>& ids, std_msgs::ColorRGBA& color, 
    std::string frame_id, visualization_msgs::MarkerArray& marker_array
){

    // Populate the MarkerArray with text markers for each point
    for (size_t i = 0; i < points.size()/4; ++i)
    {
        for (size_t corner_idx = 0; corner_idx < 4; corner_idx++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id; // Set the frame ID according to your needs
            marker.header.stamp = ros::Time::now();
            // marker.ns = "";
            marker.id = ids.at(i)*10+corner_idx;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = points.at(i*4+corner_idx)(0);
            marker.pose.position.y = points.at(i*4+corner_idx)(1);
            marker.pose.position.z = points.at(i*4+corner_idx)(2);
            marker.pose.orientation.w = 1.0;
            double scale = 0.01;
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.color=color;
            marker.text = std::to_string(ids.at(i))+"_"+std::to_string(corner_idx);

            marker_array.markers.push_back(marker);
        }
    }

}

void marker_visualization(
    std::vector<std::string>& names, 
    std::vector<std::vector<Eigen::VectorXf>>& res_points_arucos, 
    std::vector<std::vector<int>>& res_ids, 
    std::vector<std::vector<Eigen::VectorXf>>& res_triangulated_points,
    visualization_msgs::MarkerArray& triangulated_markers,
    visualization_msgs::MarkerArray& local_markers,
    std::string &frame_id
){
    for(int i =0; i<names.size(); i++)
    {
        std_msgs::ColorRGBA color;
        color.r = 1.;
        color.a = 0.5;
        add_markers(
            res_points_arucos.at(i), res_ids.at(i), color, names.at(i), 
            local_markers
        );
        color.g = 1.;
        add_markers(
            res_triangulated_points.at(i), res_ids.at(i), color, frame_id, 
            triangulated_markers
        );
        
    }
};

visualization_msgs::Marker gen_text_marker(
    std::string frame_id,
    std_msgs::ColorRGBA color,
    int id
){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id=id;
    marker.color=color;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.text = std::to_string(id);
    marker.scale.z = 0.03;  // Only Z scale is used for text size
    // Lifetime of the marker (-1 means it will stay indefinitely)
    // marker.lifetime = ros::Duration(0);
    return marker;
};


visualization_msgs::Marker gen_line_marker(
    const std::string& frame_id, 
    std_msgs::ColorRGBA color,
    double line_width
){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = line_width; 
    marker.color=color;
    marker.pose.orientation.w=1;
    // Lifetime of the marker (-1 means it will stay indefinitely)
    // marker.lifetime = ros::Duration(0);
    return marker;
};

void gen_aruco_msgs(
    std::vector<Eigen::VectorXf>& points, std::vector<int>& ids,
    std::string frame_id, std_msgs::ColorRGBA color,
    keiko_msgs::ArucoMarkers3d &arucomarkers3d, 
    visualization_msgs::MarkerArray& marker_array
){

    double line_width = 0.005;

    int marker_count=0;
    visualization_msgs::Marker lines_list = gen_line_marker(
        frame_id,color,line_width);
        
        std::cout<<"n_ids: "<<ids.size()<<std::endl;
        for (std::size_t id_idx=0; id_idx<ids.size(); id_idx++){
        std::vector<geometry_msgs::Point> corners;
        arucomarkers3d.ids.push_back(ids.at(id_idx));
        float x = 0;
        float y = 0;
        float z = 0;
        for(std::size_t cidx=0; cidx<4; cidx++){
            x+=points.at(id_idx*4+cidx)(0);
            y+=points.at(id_idx*4+cidx)(1);
            z+=points.at(id_idx*4+cidx)(2);
            geometry_msgs::Point p;
            p.x = points.at(id_idx*4+cidx)(0);
            p.y = points.at(id_idx*4+cidx)(1);
            p.z = points.at(id_idx*4+cidx)(2);
            arucomarkers3d.corners.push_back(p);
            corners.push_back(p);
        }
        // set text
        visualization_msgs::Marker m_text = gen_text_marker(
            frame_id, color, ids.at(id_idx)
        );
        m_text.id=marker_count;
        m_text.pose.position.x = x/4;
        m_text.pose.position.y = y/4;
        m_text.pose.position.z = z/4;
        marker_array.markers.push_back(m_text);
        marker_count+=1;
        // line points
        lines_list.points.push_back(corners.at(0));
        lines_list.points.push_back(corners.at(1));

        lines_list.points.push_back(corners.at(1));
        lines_list.points.push_back(corners.at(2));

        lines_list.points.push_back(corners.at(2));
        lines_list.points.push_back(corners.at(3));

        lines_list.points.push_back(corners.at(3));
        lines_list.points.push_back(corners.at(0));
    }
    lines_list.id=marker_count;
    marker_array.markers.push_back(lines_list);
}


}