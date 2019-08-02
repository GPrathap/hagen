#include "../src/include/trajectory_estimator_node.h"


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
    ros::init (argc, argv, "trajectory_estimator_node");
    kamaz::hagen::TrajectoryEstimatorNode trajectory_estimator_core;
    trajectory_estimator_core.init();
    BOOST_LOG_TRIVIAL(info) <<  FCYN("Start trajectory estimator node ")
                            << FCYN("with ") << 2 << " ROS Async Spinners";
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown(); 
    ROS_INFO("Terminate Treajectory Estimator Node");
    return EXIT_SUCCESS;
 }

