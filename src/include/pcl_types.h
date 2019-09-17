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
