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

#ifndef LIDAR_SUBNODE_HAGEN_H_
#define LIDAR_SUBNODE_HAGEN_H_

#include <time.h>
#include "lidar_process_subnode.h"
#include <fstream>

namespace kamaz {
namespace hagen {


class Hagen{
 public:
  Hagen();
  ~Hagen() = default;

  bool processing_object_detection = true;
  bool processing_trjectory_estimation = true;
  
  void onPointCloud(const sensor_msgs::PointCloud2& msg);
  void onImage(const sensor_msgs::CompressedImageConstPtr& ms);
  kamaz::hagen::LidarProcessSubnode lidar_process_subnode;

  std::ofstream time_calculation;


};
}  // namespace hagen
}  // namespace kamaz

#endif 
