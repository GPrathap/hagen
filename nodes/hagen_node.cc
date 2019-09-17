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

