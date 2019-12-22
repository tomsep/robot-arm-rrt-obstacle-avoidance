## CMDS
catkin_make -DCMAKE_BUILD_TYPE=Debug
catkin_make run_tests -DCMAKE_BUILD_TYPE=Debug

rosrun xacro xacro --inorder '/media/tomi/Data/documents/software/advanced_robotics_course/catkin_ws/src/arm-control/elfin/elfin_description/urdf/elfin3.urdf.xacro' >> urdf.urdf

rostopic pub -1 /elfin/workspace_obstacle_avoidance/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [-2.2084,   64.9009,   91.0931,    2.7517,   63.9848,   81.7514]}"

rostopic pub -1 /elfin/workspace_obstacle_avoidance/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [-0.68006,   -1.76934,   63.14618,    1.22386,   25.14582,   82.83806]}"


roslaunch rrt_solver rrt.launch

## Prerequisite
Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-kinetic-effort-controllers
   
## Example run

	roslaunch elfin_launch workspace_obstacle_avoidance.launch
	
	# In another terminal
	roslaunch rrt_solver rrt.launch

	rostopic pub -1 /elfin/workspace_obstacle_avoidance/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,   0,   90,    0,   0,  0]}"

	rostopic pub -1 /elfin/workspace_obstacle_avoidance/command_jog std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,   0,   0,    0,   0,  0]}"

## Path optimization
Go through the path (vector of q's) by skipping every other q and try collisions between them. pop all succesfully skipped q's. Do until all
tries fail.

Popping could be done using indices in single pass using reverse iteration.

How to remember which connectios already failed? Boolean vector alongside?

**Greedy** tests:
try connecting first and last straight away. (if not done elsewhere already)

Configurable skip-ratio?

## Collision checking
First interpolate greedily e.g. 0.2 intervals. Then if success use finer interpolation.

Interpolation must adapt to configuration distance.

Collision checking itself:
1. Compute sphere center points using fk. 
2 for each test collision against every obstacle.

Reverse order testing is likely better than forward.

Collision volumes

coll vol struct: solver, radius
every collision vol is tested agaisnt each obstacle

## Tree exploration

1. q_rand
2. clamp(q_rand)
3. Test collision
4. Add if collision free
5. Repeat for tree2

after certain iterations try merging
1. for each tree1 compute distance to all tree2 q's
2. save all with D <\ some limit
3. For saved ones collision check. Return path on success.
4. Else remember tried combinations and return to tree expansions step

Remembering tried combinations:
For each tree keep two vectors. Old and new nodes.
New nodes are those newer tried.
New nodes are tested on all opposing tree nodes (old and new)
and then added to the old nodes afterwards.
	