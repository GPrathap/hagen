#ifndef _PCL_TYPES_H_
#define _PCL_TYPES_H_

#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace kamaz {
namespace hagen {

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PointCloudPtr;

}}

#endif  
