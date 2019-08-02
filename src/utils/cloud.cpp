#include "cloud.h"

namespace kamaz {

namespace hagen{

void Cloud::SetProjectionPtr(typename CloudProjection::Ptr proj_ptr) {
  _projection = proj_ptr;
}

void Cloud::SetProjection(){
  _projection = _projection->Clone();
  _projection->InitFromPoints(point_cloud_ptr, cloud_depth_mapper);
}

void Cloud::InitProjection(const ProjectionParams& params) {
  if (!_projection) {
    _projection = CloudProjection::Ptr(new SphericalProjection(params));
  }
  if (!_projection) {
    fprintf(stderr, FBLU("ERROR: failed to initalize projection.\n"));
    return;
  }
  _projection = _projection->Clone();
  _projection->InitFromPoints(point_cloud_ptr, cloud_depth_mapper); 
}

} 
}
