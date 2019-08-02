#include "lidar_process_subnode.h"

namespace kamaz {
namespace hagen {

using Eigen::Affine3d;
using Eigen::Matrix4d;
using std::vector;
using std::string;
using std::map;
using std::mutex;
using std::lock_guard;
using cv::Mat;
using cv::DataType;
using std::to_string;


LidarProcessSubnode::LidarProcessSubnode(): 
  world_frame_id("/world"), base_frame_id("velodyne")
{
  
}

bool LidarProcessSubnode::initInternal() {
  if (inited_) {
    return true;
  }
  if (!init()) {
    ROS_ERROR("failed to init algorithm plugin.");
    return false;
  }
  return true;
}

void LidarProcessSubnode::onPointCloud(const sensor_msgs::PointCloud2& cloud) {

  // BOOST_LOG_TRIVIAL(info) << FCYN("Process onPointCloud");
  if (!inited_) {
     BOOST_LOG_TRIVIAL(warning) <<  FBLU("The LidarProcessSubnode has not been init");
    return;
  }

  pcl::PointCloud<PCLPoint> pc;
  if(cloud.width<1){
    return;
  }

  pcl::fromROSMsg(cloud, pc);
  Eigen::Matrix4f sensorToWorld;
  tf::StampedTransform sensor_to_world_tf;
  try {
    m_tfListener.lookupTransform(world_frame_id, cloud.header.frame_id
          , ros::Time(0), sensor_to_world_tf);
  } catch(tf::TransformException& ex){
    BOOST_LOG_TRIVIAL(warning) << FBLU("Transform error of sensor data") ;
    ros::Duration(0.25).sleep();
    return;
  }

  Cloud::Ptr internal_cloud_ptr_current_ptr;
  internal_cloud_ptr_current_ptr.reset(new Cloud());

  tf::Transform tf;
  tf = common_utils.get_tf_from_stamped_tf(sensor_to_world_tf);
  tf::Matrix3x3 tfR;
  Eigen::Matrix3d rotation;
  tfR = tf.getBasis();
  tf::matrixTFToEigen(tfR, rotation);
  Eigen::Matrix4f sensor_to_world_rotation = Eigen::Matrix4f::Identity();
  sensor_to_world_rotation.block<3,3>(0,0) = rotation.cast<float>();

  auto in_cloud = boost::make_shared<pcl::PointCloud<PCLPoint>>(pc);
  pcl::transformPointCloud(*in_cloud, *in_cloud, sensor_to_world_rotation);
  separate_ground_and_non_ground(internal_cloud_ptr_current_ptr, *in_cloud);
  internal_cloud_ptr_->push_back(internal_cloud_ptr_current_ptr);
  internal_tf_transform_frame->push_back(std::move(sensor_to_world_tf));

  if((int)internal_cloud_ptr_->size() == number_point_clouds_to_be_merged){
    auto cloud_buffer = *internal_cloud_ptr_;
    auto transformation_buffer = *internal_tf_transform_frame;
    Cloud::Ptr cloud_ptr_current_ptr;
    cloud_ptr_current_ptr = cloud_buffer[number_point_clouds_to_be_merged-1];
    PointCloudPtr point_cloud_ptr;
    PointCloudPtr point_cloud_ground_ptr;
    PointCloudPtr point_cloud_non_ground_ptr;

    auto tf_k_end = common_utils.get_tf_from_stamped_tf(transformation_buffer[number_point_clouds_to_be_merged-1]);
    tf::Vector3 tf_vec_k_end = tf_k_end.getOrigin();

    for(int k=number_point_clouds_to_be_merged-2; k = 0; k--){
      auto tf_k = common_utils.get_tf_from_stamped_tf(transformation_buffer[k]);
      tf::Vector3 tf_vec_k = tf_k.getOrigin();
      auto diff_vec =  tf_vec_k - tf_vec_k_end;
      Eigen::Matrix4f transtation = Eigen::Matrix4f::Identity();
      transtation(0, 3) = diff_vec.getX();
      transtation(1, 3) = diff_vec.getY();
      transtation(2, 3) = diff_vec.getZ();

      point_cloud_ptr.reset(new pcl::PointCloud<PCLPoint>());
      point_cloud_ground_ptr.reset(new pcl::PointCloud<PCLPoint>());
      point_cloud_non_ground_ptr.reset(new pcl::PointCloud<PCLPoint>());

      pcl::transformPointCloud(*(cloud_buffer[k]->point_cloud_ptr)
      , *(point_cloud_ptr), transtation);
      pcl::transformPointCloud(*(cloud_buffer[k]->point_cloud_ground_plane)
      , *(point_cloud_ground_ptr), transtation);
      pcl::transformPointCloud(*(cloud_buffer[k]->point_cloud_non_ground_plane)
      , *(point_cloud_non_ground_ptr), transtation);

      // pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

      *(cloud_ptr_current_ptr->point_cloud_ground_plane) += *(point_cloud_ground_ptr);
      *(cloud_ptr_current_ptr->point_cloud_non_ground_plane) += *(point_cloud_non_ground_ptr);
      *(cloud_ptr_current_ptr->point_cloud_ptr) += *(point_cloud_ptr);
    }

  tf::Transform tf;
  tf = common_utils.get_tf_from_stamped_tf(sensor_to_world_tf);
  tf::Matrix3x3 tfR;
  Eigen::Matrix3d rotation;
  tfR = tf.getBasis();
  tf::matrixTFToEigen(tfR, rotation);
  
  Eigen::Matrix4f world_to_sensor_rotation = Eigen::Matrix4f::Identity();
  world_to_sensor_rotation.block<3,3>(0,0) = (rotation.cast<float>()).transpose();

  pcl::transformPointCloud(*(cloud_ptr_current_ptr->point_cloud_non_ground_plane)
  , *(cloud_ptr_current_ptr->point_cloud_non_ground_plane), world_to_sensor_rotation);

  pcl::transformPointCloud(*(cloud_ptr_current_ptr->point_cloud_ground_plane)
  , *(cloud_ptr_current_ptr->point_cloud_ground_plane), world_to_sensor_rotation);

  sensor_msgs::PointCloud2 point_cloud_ground_plane_cloud;
  sensor_msgs::PointCloud2 point_cloud_non_ground_plane_cloud;
  pcl::toROSMsg(*(cloud_ptr_current_ptr->point_cloud_non_ground_plane).get(), point_cloud_non_ground_plane_cloud);
  pcl::toROSMsg(*(cloud_ptr_current_ptr->point_cloud_ground_plane).get(), point_cloud_ground_plane_cloud);
  point_cloud_ground_plane_cloud.header.frame_id = base_frame_id;
  point_cloud_non_ground_plane_cloud.header.frame_id = base_frame_id;
  point_cloud_ground_plane_publisher.publish(point_cloud_ground_plane_cloud);
  point_cloud_non_ground_plane_publisher.publish(point_cloud_non_ground_plane_cloud);
  
  _counter++;
  
  }

  BOOST_LOG_TRIVIAL(info) <<  FCYN("Finished processing point cloud");
  return;
}



void LidarProcessSubnode::separate_ground_and_non_ground(Cloud::Ptr& cloud_ptr_current_ptr
  , pcl::PointCloud<PCLPoint> pc){

        auto in_cloud = boost::make_shared<pcl::PointCloud<PCLPoint>>(pc);
        cloud_ptr_current_ptr->point_cloud_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
        cloud_ptr_current_ptr->point_cloud_non_ground_plane.reset(new pcl::PointCloud<PCLPoint>());
        cloud_ptr_current_ptr->point_cloud_ptr.reset(new pcl::PointCloud<PCLPoint>());
        cloud_ptr_current_ptr->point_cloud_ptr = in_cloud;
        cloud_ptr_current_ptr->time_stamp = ros::Time::now();

        try{
          cloud_ptr_current_ptr->InitProjection(*proj_params_ptr);
        }catch (const std::length_error& le) {
          std::cerr << FBLU("Error:point cloud is empty...") << le.what() << std::endl;;
          return;
        }
        
        // auto startTime = ros::WallTime::now();
        BOOST_LOG_TRIVIAL(info) << FCYN("Number of points in the cloud") << cloud_ptr_current_ptr->point_cloud_ptr->points.size();
        
        // DepthGroundRemover::HagenFilterOptions<cloud_ptr_current_ptr
        // , number_of_components, ground_remove_angle, _counter> hagen_filter_options;

    depth_ground_remover->options.bin_size = number_of_components;
    depth_ground_remover->options.ground_remove_angle = ground_remove_angle;
    depth_ground_remover->options.step = 5;
    depth_ground_remover->options.depth_threshold = 1.0f;
    depth_ground_remover->options.window_size = smooth_window_size;
    depth_ground_remover->options.kernel_size = smooth_window_size;
    depth_ground_remover->options.depth_expiration_time = 1.0;
    depth_ground_remover->execute<Cloud::Ptr>(cloud_ptr_current_ptr, _counter);
    _counter++;
  }





bool LidarProcessSubnode::init() {

  proj_params_ptr = kamaz::hagen::ProjectionParams::VLP_16(object_avoidance_zone);
  depth_ground_remover = new DepthGroundRemover(*proj_params_ptr);

  cloud_ptr_ = std::make_shared<boost::circular_buffer<Cloud::Ptr>>(size_of__message_buffer);
  tf_transform_frame = std::make_shared<boost::circular_buffer<tf::StampedTransform>>(size_of__message_buffer);
  
  internal_cloud_ptr_ = std::make_shared<boost::circular_buffer<Cloud::Ptr>>(number_point_clouds_to_be_merged);
  internal_tf_transform_frame = std::make_shared<boost::circular_buffer<tf::StampedTransform>>(number_point_clouds_to_be_merged);
  
  common_utils.voxel_side_length = voxel_side_length;
  common_utils.world_frame_id = world_frame_id;
  inited_ = true;
  return true;
}

}  
}  
