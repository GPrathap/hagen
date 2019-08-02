#ifndef LIDAR_SUBNODE_HAGEN_H_
#define LIDAR_SUBNODE_HAGEN_H_


#include "lidar_process_subnode.h"

namespace kamaz {
namespace hagen {


class Hagen{
 public:
  Hagen();
  ~Hagen() = default;

  bool processing_object_detection = true;
  bool processing_trjectory_estimation = true;
  
  void onPointCloud(const sensor_msgs::PointCloud2& msg);
  
  kamaz::hagen::LidarProcessSubnode lidar_process_subnode;

};
}  // namespace hagen
}  // namespace kamaz

#endif 
