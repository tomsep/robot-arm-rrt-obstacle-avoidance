![](https://img.shields.io/badge/implementation-C%2B%2B-blue) ![](https://img.shields.io/badge/license-MIT-green) ![](https://img.shields.io/badge/ROS-Kinetic-blue)
# 6-DoF Robot Arm Obstacle avoidance

The objective is to find **collision free path** from _start_ to _goal_ position even in the presence of multiple obstacles. RRT based (rapidly-exploring random tree) method is used to explore the collision free movement space.

ROS controller template and [Elfin robot](http://wiki.ros.org/Robots/Elfin) assets were adapted from [https://github.com/modulabs/arm-control](https://github.com/modulabs/arm-control).


**Implemented for this project**:

* Pathfinding using rapidly-exploring random tree (RRT)
* Collision detection between the robot and the obstacles
* Motion controller
* Visualizations (collision volume, obstacles, point cloud)

![demogif](https://i.imgur.com/bBwjW1I.gif)

Fig 1: Robot finding its way past the solid green obstacles.

Fig 1 demonstrates how the space is explored using two trees, one starting at _start_ position and the other at _goal_. Connections between the trees' points are not visualized because in 3D these connections would be non-linear.

The green see-through field is the robot's approximate collision volume.
This is the volume that is considered when computing collisions with the obstacles.

Solid green spheres are the obstacles that must be avoided when moving towards _goal_ position.
When collision free path is found it is forwarded to the motion controller and the robot begins moving.


## Pathfinding using rapidly-exploring random tree (RRT)


![RRT 2D example](https://upload.wikimedia.org/wikipedia/commons/6/62/Rapidly-exploring_Random_Tree_%28RRT%29_500x373.gif)

Fig 2: Tree expansion in 2D. Animation from [Wikipedia](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree).

This project uses **two trees**: one begins at _start_ position
and the other at _goal_ position. Both trees are expanded
for set amount of iterations. Also known as bidirectional RRT.

Expansion is implemented by randomly generating candidate robot poses. The algorithm searches for its closest tree branch and tries to connect to it. The connection is tested for collission. If the whole connection is collision free then the candidate pose is added to the tree.

After the expansion phase an attempt to merge these two
trees is made. If any collision free path is found between
these trees the trees are merged to form a single tree and path
from _start_ to _goal_ can be generated. Following this path guarantees
collision free movement.

## Installation & building
Install gazebo-ros-pkgs and gazebo-ros-control
`$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control`

Install effort-controllers to use torque-control interface
`$ sudo apt-get install ros-kinetic-effort-controllers`
    
Build project
`$ catkin_make -DCMAKE_BUILD_TYPE=Debug`
	
To build and run tests
`$ catkin_make run_tests -DCMAKE_BUILD_TYPE=Debug`

   
## Example run
	
	$ source devel/setup.bash
	$ roslaunch elfin_launch manipulator_controller.launch
	
	# In another terminal
	$ source devel/setup.bash
	$ roslaunch rrt_solver rrt.launch

	# Set joint space target (joint degrees)
	rostopic pub -1 /elfin/manipulator_controller/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,   0,   120, 0, -30, 0]}"
	
	# Target to get back to initial position
	rostopic pub -1 /elfin/manipulator_controller/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,   0,   0,    0,   0,  0]}"

