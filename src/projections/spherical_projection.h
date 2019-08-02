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
