#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>

#include "../src/lidar_subnode/lidar_process_subnode.h"
#include "../src/include/ground_removal_filter_node.h"
#include <iostream>
#include <string>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread/thread.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace kamaz {
namespace hagen{
    
    GroundRemovalFilterNode::GroundRemovalFilterNode()
        : nh_("~")
    {
    }

    GroundRemovalFilterNode::~GroundRemovalFilterNode() {

    };

    void GroundRemovalFilterNode::init_logging(std::string logging_dir)
    {
        const std::string COMMON_FMT("[%TimeStamp%][%Severity%]:  %Message%");
        boost::log::register_simple_formatter_factory< boost::log::trivial::severity_level, char >("Severity");

        // Output message to console
        boost::log::add_console_log(
            std::cout,
            boost::log::keywords::format = COMMON_FMT,
            boost::log::keywords::auto_flush = true
        );

        // Output message to file, rotates when file reached 1mb or at midnight every day. Each log file
        // is capped at 1mb and total is 20mb
        boost::log::add_file_log (
            boost::log::keywords::file_name = logging_dir + "/sample_%3N.log",
            boost::log::keywords::rotation_size = 1 * 1024 * 1024,
            boost::log::keywords::max_size = 20 * 1024 * 1024,
            boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
            boost::log::keywords::format = COMMON_FMT,
            boost::log::keywords::auto_flush = true
        );

        boost::log::add_common_attributes();

        // Only output message with INFO or higher severity in Release
    #ifndef _DEBUG
        boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::info
        );
    #endif

    }

    void GroundRemovalFilterNode::init(){
       
        nh_.param("point_cloud_topic_name", hagen.lidar_process_subnode.point_cloud_topic, hagen.lidar_process_subnode.point_cloud_topic);
        nh_.param("frame_id", hagen.lidar_process_subnode.world_frame_id, hagen.lidar_process_subnode.world_frame_id);
        nh_.param("child_frame_id", hagen.lidar_process_subnode.base_frame_id, hagen.lidar_process_subnode.base_frame_id);
        nh_.param("object_avoidance_zone", hagen.lidar_process_subnode.object_avoidance_zone, hagen.lidar_process_subnode.object_avoidance_zone);
        nh_.param("ground_remove_angle", hagen.lidar_process_subnode.ground_remove_angle, hagen.lidar_process_subnode.ground_remove_angle);
        nh_.param("number_point_clouds_to_be_merged", hagen.lidar_process_subnode.number_point_clouds_to_be_merged, hagen.lidar_process_subnode.number_point_clouds_to_be_merged);
        nh_.param("number_of_components", hagen.lidar_process_subnode.number_of_components, hagen.lidar_process_subnode.number_of_components);
        init_logging(logging_dir);

        ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

        BOOST_LOG_TRIVIAL(info) << "Point cloud topic " << hagen.lidar_process_subnode.point_cloud_topic;
       
        hagen.lidar_process_subnode.point_cloud_ground_plane_publisher = node->advertise<sensor_msgs::PointCloud2> ("/hagen/point_cloud_ground_plane", 1);
        hagen.lidar_process_subnode.point_cloud_non_ground_plane_publisher = node->advertise<sensor_msgs::PointCloud2> ("/hagen/point_cloud_non_ground_plane", 1);
        sub_on_point_cloud = node->subscribe(hagen.lidar_process_subnode.point_cloud_topic, 10, &kamaz::hagen::Hagen::onPointCloud, &hagen);

        hagen.lidar_process_subnode.initInternal();
    }

}  

}
