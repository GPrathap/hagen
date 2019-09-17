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

#ifndef PROJECTION_SPHERICAL_PROJECTION_H_
#define PROJECTION_SPHERICAL_PROJECTION_H_

#include <opencv/cv.h>

#include <vector>

#include "cloud_projection.h"
#include "projection_params.h"
#include "../utils/cloud.h"

namespace kamaz {

namespace hagen{

class SphericalProjection : public CloudProjection {
 public:
  explicit SphericalProjection(const ProjectionParams& projection_params)
      : CloudProjection(projection_params) {}

  void InitFromPoints(kamaz::hagen::PointCloudPtr point_cloud_ptr, std::map<int, kamaz::hagen::PCLPoint>& depth_angle_mappe) override;
  typename CloudProjection::Ptr Clone() const override;
  virtual ~SphericalProjection() {}
  
};

}  
}
#endif  
