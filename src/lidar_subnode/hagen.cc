#include "hagen.h"

namespace kamaz {
namespace hagen {


  Hagen::Hagen(){
  }

  void Hagen::onPointCloud(const sensor_msgs::PointCloud2& message){
      lidar_process_subnode.onPointCloud(message);
  }
  
  }  // namespace hagen
}  // namespace kamaz
