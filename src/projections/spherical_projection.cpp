// Copyright (C) 2019  Geesara Kulathunga, R. Fedorenko, University of Innopolis, Russia
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn
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

#include "spherical_projection.h"

#include <vector>

namespace kamaz {

namespace hagen{

void SphericalProjection::InitFromPoints(kamaz::hagen::PointCloudPtr point_could_ptr, std::map<int,  kamaz::hagen::PCLPoint>& depth_cloud_mapper){
  
  int num_pts = static_cast<int>(point_could_ptr->points.size());
  if (num_pts < 1) {
    throw std::length_error("_data size is < 1");
  }
  for (int index = 0; index < num_pts; ++index) {
    const auto& point = point_could_ptr->points[index];
    float dist_to_sensor = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (dist_to_sensor < 0.4f or dist_to_sensor > this->_params.getMaxDistance()) {
      // std::cout<< dist_to_sensor << std::endl; 
      continue;
    }

    auto angle_rows = Radians::FromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Radians::FromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.RowFromAngle(angle_rows);
    size_t bin_cols = this->_params.ColFromAngle(angle_cols);

    //std::cout<< "bin-->" << bin_rows << " : " << bin_cols << " : " << std::endl;
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    std::tuple<int, int> index_(bin_rows, bin_cols); 
    depth_cloud_mapper[index] = point;
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
    }
  }
  FixDepthSystematicErrorIfNeeded();
}

typename CloudProjection::Ptr SphericalProjection::Clone() const {
  return typename CloudProjection::Ptr(new SphericalProjection(*this));
}

}  
}
