#ifndef _UTILS_COMMON_UTILS_H_
#define _UTILS_COMMON_UTILS_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include <cnpy.h>
#include <random>
#include <algorithm>
#include <list>
#include <numeric>
#include <unordered_map>
#include <memory>
#include <type_traits>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <deque>
#include <atomic>

#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include "../include/colours.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>


namespace kamaz {
  namespace hagen{
class CommonUtils {
 public:
  
  CommonUtils() = default;
  ~CommonUtils() = default;
  void generate_samples_from_ellipsoid(Eigen::MatrixXf covmat
            , Eigen::Matrix3f rotation_mat, Eigen::VectorXf cent
            , Eigen::MatrixXf& container);
  
  void get_roration_matrix(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Matrix3f& r);
  visualization_msgs::Marker create_marker_point(Eigen::VectorXf _point_on_path
        , Eigen::MatrixXf covmat, int id_, std::string name_space);
  visualization_msgs::Marker create_marker_point(Eigen::VectorXf _point_on_path
        , Eigen::MatrixXf covmat, Eigen::Quaternion<double> q, int id_
        , std::string name_space);
  visualization_msgs::Marker create_marker_point(Eigen::VectorXf _point_on_path
        , ColorRGBA color_of_qupter, int id_, std::string name_space);
  
  geometry_msgs::PoseStamped constructPoseStamped(Eigen::VectorXf path_position);
  void printStampedTf(tf::StampedTransform sTf);
  void printTf(tf::Transform tf);
  void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg);
  tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
  void get_point_on_the_trajectory(Eigen::VectorXf way_point
      , Eigen::VectorXf start_point,  Eigen::VectorXf& path_position);

  float voxel_side_length;
  Eigen::Vector3f init_min_point;
  std::string world_frame_id;
};

}  
}
#endif  // _UTILS_COMMON_UTILS_H_
