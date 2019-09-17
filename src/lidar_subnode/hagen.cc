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

#include "hagen.h"

namespace kamaz {
namespace hagen {


  Hagen::Hagen(){
    time_calculation.open("/home/ros_workspace/src/groud_removal_filter/src/time_diff.txt", std::ios_base::app);
  }

  void Hagen::onPointCloud(const sensor_msgs::PointCloud2& msg){
      const clock_t begin_time = clock();
      lidar_process_subnode.onPointCloud(msg);
      time_calculation <<  (float( clock () - begin_time ) /  CLOCKS_PER_SEC) << "\n"; 

  }

  void Hagen::onImage(const sensor_msgs::CompressedImageConstPtr& msg){
      lidar_process_subnode.onImage(msg);
  }
  
  }  // namespace hagen
}  // namespace kamaz
