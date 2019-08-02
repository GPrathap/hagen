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
