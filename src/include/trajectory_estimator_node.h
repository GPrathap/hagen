#ifndef TRAJECTORY_ESTIMATOR_TRAJECTORY_ESTIMATOR_H_
#define TRAJECTORY_ESTIMATOR_TRAJECTORY_ESTIMATOR_H_

#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <time.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <iostream>

#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/attributes/mutable_constant.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <vector>
#include <string>

#include "../lidar_subnode/hagen.h"

namespace kamaz{
    namespace hagen{
        class TrajectoryEstimatorNode{
        public:
            TrajectoryEstimatorNode();
            ~TrajectoryEstimatorNode();
            
            void init();
            void init_logging(std::string logging_dir);
            bool inited_ = false;

            ros::NodeHandle nh_;
            kamaz::hagen::Hagen hagen;
            // private ROS node handle
            ros::Subscriber sub_on_point_cloud;
            ros::Subscriber sub_on_gps;

            ros::Subscriber attitudeSub;
            ros::Subscriber gpsSub;
            ros::Subscriber flightStatusSub;
            ros::Subscriber displayModeSub;
            ros::Subscriber localPosition;
            ros::Subscriber odometrySub;
            ros::Subscriber projectedTrajectorySub;
            ros::Subscriber trajectorySegmentsSub;
            ros::Subscriber gimbalAngleSub;
            ros::Subscriber rc_sub;
            ros::Subscriber gpsReferencePoseSub;
            ros::Subscriber gimbalSub;

            std::thread object_detection_thread;
            std::thread estimae_trajectory_thread;
            std::thread try_to_avoid_obstacles_thread;

            std::string logging_dir;
        protected:

        };
     }

}

#endif 
