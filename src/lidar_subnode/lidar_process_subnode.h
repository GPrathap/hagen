#ifndef LIDAR_SUBNODE_LIDAR_PROCESS_SUBNODE_H_
#define LIDAR_SUBNODE_LIDAR_PROCESS_SUBNODE_H_

#include <future>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/circular_buffer.hpp>

#include "../utils/common_utils.h"
#include "../include/pcl_types.h"
#include "../ground_removal/depth_ground_remover.h"
#include "../projections/spherical_projection.h"
#include "../projections/projection_params.h"
#include "../utils/radians.h"
#include "../utils/cloud.h"
#include "../include/colours.h"

namespace kamaz {
namespace hagen {

using PointCloudT = sensor_msgs::PointCloud2;

class LidarProcessSubnode{
 public:
  LidarProcessSubnode();
  ~LidarProcessSubnode() = default;

  void onPointCloud(const sensor_msgs::PointCloud2& message);


  std::string point_cloud_topic;
  bool inited_ = false;
  bool initInternal();

  tf::TransformListener m_tfListener;
  std::string world_frame_id; // the world frame
  std::string base_frame_id; // base of the robot for ground plane filtering

  float object_avoidance_zone;
  float voxel_side_length = 0.2f; // 20 cm voxel size

 
  float obstacle_width = 2;

  float ground_remove_angle = 10;
 
 
  int number_of_components = 3;
  int number_point_clouds_to_be_merged = 3;
  
  ros::Publisher point_cloud_ground_plane_publisher;
  ros::Publisher point_cloud_non_ground_plane_publisher;

 private: 
  bool init();
  
  void separate_ground_and_non_ground(Cloud::Ptr& cloud_ptr_current_ptr
  , pcl::PointCloud<PCLPoint> pc);
  
  std::shared_ptr< boost::circular_buffer<Cloud::Ptr>> cloud_ptr_;
  std::shared_ptr< boost::circular_buffer<tf::StampedTransform>> tf_transform_frame;
  std::shared_ptr< boost::circular_buffer<Cloud::Ptr>> internal_cloud_ptr_;
  std::shared_ptr< boost::circular_buffer<tf::StampedTransform>> internal_tf_transform_frame;
  int size_of__message_buffer = 2;

  int min_cluster_size = 100;
  int max_cluster_size = 25000;
  int smooth_window_size = 7;
  mutable int _counter = 0;
  Eigen::Vector3f init_min_point, init_max_point;
  pcl::PointXYZ current_min_point;
  Eigen::VectorXf center_position;
  float x_y_size_of_ellipsoid = 4.0;
  std::map<int, int>  obstacle_orientations;
  DepthGroundRemover* depth_ground_remover;
  std::unique_ptr<ProjectionParams> proj_params_ptr;
  CommonUtils common_utils;
  Eigen::VectorXf map_dimensions;
  Eigen::Vector3f x_init;

};
}  // namespace hagen
}

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
