// Copyright (C) 2019  Geesara Kulathunga, R. Fedorenko, University of Innopolis, Russia
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "common_utils.h"

namespace kamaz {
namespace hagen{


  void CommonUtils::get_roration_matrix(Eigen::Vector3f a
      , Eigen::Vector3f b, Eigen::Matrix3f& r){
        a = a/a.norm();
        float b_norm = b.norm();
        b = b/b_norm;
        Eigen::Vector3f v = a.cross(b);
        float s = v.norm();
        float c = a.dot(b);
        Eigen::Matrix3f vx;
        vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        r = Eigen::Matrix3f::Identity(3,3);
        if(s != 0 ){
            r = r + vx + vx*vx*((1-c)/std::pow(s, 2));
        }
  }

  void CommonUtils::get_point_on_the_trajectory(Eigen::VectorXf way_point, Eigen::VectorXf start_point,  Eigen::VectorXf& path_position){
        path_position << way_point[0]*voxel_side_length + init_min_point[0]
        , way_point[1]*voxel_side_length + init_min_point[1]
        , way_point[2]*voxel_side_length + init_min_point[2], 1.0;
        path_position = path_position + start_point;
  }

  void CommonUtils::generate_samples_from_ellipsoid(Eigen::MatrixXf covmat, Eigen::Matrix3f rotation_mat, 
            Eigen::VectorXf cent, Eigen::MatrixXf& container){

        int ndims = container.cols();
        int npts = container.rows();
        Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
        eigensolver.compute(covmat);
        Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
        Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
        std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values; 

        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(), 
            [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{ 
                return std::get<0>(a) <= std::get<0>(b); 
        });
        int index = 0;
        for(auto const vect : eigen_vectors_and_values){
            eigen_values(index) = std::get<0>(vect);
            eigen_vectors.row(index) = std::get<1>(vect);
            index++;
        }

        Eigen::MatrixXf eigen_values_as_matrix = eigen_values.asDiagonal();

        std::random_device rd{};
        std::mt19937 gen{rd()};  
        std::uniform_real_distribution<float> dis(0, 1);
        std::normal_distribution<double> normal_dis{0.0f, 1.0f};
 
        Eigen::MatrixXf pt = Eigen::MatrixXf::Zero(npts, ndims).unaryExpr([&](float dummy){return (float)normal_dis(gen);});
        Eigen::VectorXf rs = Eigen::VectorXf::Zero(npts).unaryExpr([&](float dummy){return dis(gen);});
        Eigen::VectorXf fac = pt.array().pow(2).rowwise().sum();
        Eigen::VectorXf fac_sqrt = fac.array().sqrt();
        Eigen::VectorXf rs_pow = rs.array().pow(1.0/ndims);
        fac = rs_pow.array()/fac_sqrt.array();
        Eigen::VectorXf d = eigen_values_as_matrix.diagonal().array().sqrt();
        for(auto i(0); i<npts; i++){
            container.row(i) = fac(i)*pt.row(i).array();
            Eigen::MatrixXf  fff = (container.row(i).array()*d.transpose().array());
            Eigen::VectorXf bn = rotation_mat*fff.transpose();
            container.row(i) = bn.array() + cent.head(3).array();
        }
        // std::cout << "points: " << container << std::endl;
    }

    visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::VectorXf _point_on_path,
        Eigen::MatrixXf covmat, int id_, std::string name_space){ 
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = name_space;
        marker.id = id_;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = _point_on_path[0];
        marker.pose.position.y = _point_on_path[1];
        marker.pose.position.z = _point_on_path[2];
        marker.pose.orientation.x = 0.1;
        marker.pose.orientation.y = 0.1;
        marker.pose.orientation.z = 0.1;
        marker.pose.orientation.w = 0.2;
        marker.scale.x = covmat(0,0)*voxel_side_length;
        marker.scale.y = covmat(1,1)*voxel_side_length;
        marker.scale.z = covmat(2,2)*voxel_side_length;
        marker.color.a = 0.4;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.8;
        marker.lifetime = ros::Duration(); 
        return marker;
    }

    visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::VectorXf _point_on_path, ColorRGBA color_of_qupter, int id_, std::string name_space){ 
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.pose.position.x = _point_on_path[0];
        marker.pose.position.y = _point_on_path[1];
        marker.pose.position.z = _point_on_path[2];
        marker.color.r = color_of_qupter.r;
        marker.color.g = color_of_qupter.g;
        marker.color.b = color_of_qupter.b;
        marker.color.a = 1.0;
        marker.ns = name_space;
        marker.id = id_;
        marker.lifetime = ros::Duration();
        return marker;
    }

    visualization_msgs::Marker CommonUtils::create_marker_point(Eigen::VectorXf _point_on_path,
        Eigen::MatrixXf covmat, Eigen::Quaternion<double> q, int id_, std::string name_space){ 
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = name_space;
        marker.id = id_;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = _point_on_path[0];
        marker.pose.position.y = _point_on_path[1];
        marker.pose.position.z = _point_on_path[2];
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = covmat(0,0)*voxel_side_length;
        marker.scale.y = covmat(1,1)*voxel_side_length;
        marker.scale.z = covmat(2,2)*voxel_side_length;
        marker.color.a = 0.4;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(); 
        return marker;
    }

    geometry_msgs::PoseStamped CommonUtils::constructPoseStamped(Eigen::VectorXf path_position){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = world_frame_id;
        pose.pose.position.x = path_position[0];
        pose.pose.position.y = path_position[1];
        pose.pose.position.z = path_position[2];
        return pose;
    }

    void CommonUtils::printStampedTf(tf::StampedTransform sTf){
        tf::Transform tf;
        BOOST_LOG_TRIVIAL(info) << "frame_id: "<<sTf.frame_id_;
        BOOST_LOG_TRIVIAL(info) << "child_frame_id: "<<sTf.child_frame_id_; 
        tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf  
        printTf(tf);       
    }

    void CommonUtils::printTf(tf::Transform tf) {
        tf::Vector3 tfVec;
        tf::Matrix3x3 tfR;
        Eigen::Matrix3d e;
        
        tf::Quaternion quat;
        tfVec = tf.getOrigin();
        BOOST_LOG_TRIVIAL(info) << "Vector from reference frame to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();
        tfR = tf.getBasis();
        BOOST_LOG_TRIVIAL(info) << "Orientation of child frame w/rt reference frame: ";
        tfVec = tfR.getRow(0);
        BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();
        tfVec = tfR.getRow(1);
        BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ();    
        tfVec = tfR.getRow(2);
        BOOST_LOG_TRIVIAL(info) << tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ(); 
        quat = tf.getRotation();
        BOOST_LOG_TRIVIAL(info) << "quaternion: " <<quat.x()<<", "<<quat.y()<<", "
        <<quat.z()<<", "<<quat.w();
        tf::matrixTFToEigen(tfR, e);
        std::cout << e << std::endl;   
    }

    tf::Transform CommonUtils::get_tf_from_stamped_tf(tf::StampedTransform sTf) {
        tf::Transform tf(sTf.getBasis(), sTf.getOrigin());
        return tf;
    }

    void CommonUtils::PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
        fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
        fprintf(stderr, "received msg   %d\n", msg->header.seq);
        fprintf(stderr, "height:        %d\n", msg->height);
        fprintf(stderr, "width:         %d\n", msg->width);
        fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
        fprintf(stderr, "fields of each point:\n");
        for (auto const& pointField : msg->fields) {
            fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
            fprintf(stderr, "\toffset:   %d\n", pointField.offset);
            fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
            fprintf(stderr, "\tcount:    %d\n", pointField.count);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
        fprintf(stderr, "point step:    %d\n", msg->point_step);
        fprintf(stderr, "row step:      %d\n", msg->row_step);
        fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
        fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
        fprintf(stderr, "=========================================\n");
    }

}
}