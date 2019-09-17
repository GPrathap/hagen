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

#ifndef _UTILS_CLOUD_H_
#define _UTILS_CLOUD_H_

#define PCL_NO_PRECOMPILE

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

#include "../projections/cloud_projection.h"
#include "../projections/spherical_projection.h"

#include "../include/pcl_types.h"
#include "../include/colours.h"

namespace kamaz {
  namespace hagen{
class Cloud {
 public:
  using Ptr = boost::shared_ptr<Cloud>;
  using ConstPtr = boost::shared_ptr<const Cloud>;

  Cloud(){}
  explicit Cloud(const Cloud& cloud);

  virtual ~Cloud() {}

  inline const typename CloudProjection::ConstPtr projection_ptr() const {
    return _projection;
  }

  inline typename CloudProjection::Ptr projection_ptr() { return _projection; }
 

  void SetProjectionPtr(typename CloudProjection::Ptr proj_ptr);

  void InitProjection(const ProjectionParams& params);

  void SetProjection();

 kamaz::hagen::PointCloudPtr point_cloud_ptr;
 kamaz::hagen::PointCloudPtr point_cloud_ground_plane;
 kamaz::hagen::PointCloudPtr point_cloud_non_ground_plane;
 std::map<int, kamaz::hagen::PCLPoint> cloud_depth_mapper;
 ros::Time time_stamp;
 protected:
  CloudProjection::Ptr _projection = nullptr;
};

}  // namespace kamaz
}
#endif  // SRC_UTILS_CLOUD_H_
