# hagen
## Installation instructions
1. Install cnpy from here https://github.com/rogersce/cnpy.git
2. Download necessary ROS packages https://github.com/GPrathap/util  with dependencies and place them in the ROS workspace
3. Download a bag file from here (https://github.com/GPrathap/dataset_for_hagen) and configure its absolute path in dji_command/launch/coords_bringup.launch

#### To run the filter 
roslaunch ground_removal_filter hagen_node.launch

#### To run the bag file and subsequent nodes
roslaunch dji_command coords_bringup.launch


# This work is under review now. Once the it is accepted, details explanation will be provided

