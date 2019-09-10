#include "../src/include/ground_removal_filter_node.h"


void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}

int main (int argc, char** argv) {
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler); 
    ros::init (argc, argv, "ground_removal_filter_node");
    kamaz::hagen::GroundRemovalFilterNode ground_removal_filter_node;
    ground_removal_filter_node.init();
    BOOST_LOG_TRIVIAL(info) <<  FCYN("Start hagen node ")
                            << FCYN("with ") << 1 << " ROS Async Spinners";
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown(); 
    ROS_INFO("Terminate Treajectory Estimator Node");
    return EXIT_SUCCESS;
 }

