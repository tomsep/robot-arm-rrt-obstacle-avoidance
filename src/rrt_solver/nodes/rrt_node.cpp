#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "rrt_solver/rrt_solver.h"
#include <Eigen/Core>
#include <vector>
#include <queue>
#include <tuple>
#include<string>


typedef std::queue<std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 1>>> cmdqueue_t;
cmdqueue_t cmd_queue;

void solve_cmd(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    const unsigned int dim = 12;
    if (msg->data.size() != dim)
    {
        ROS_ERROR_STREAM("Dimension of command " << msg->data.size() << " != " << dim << ". Ignoring.");
    } else
    {   
        Eigen::Matrix<double, 6, 1> start, goal;
        for (unsigned int i = 0; i < msg->data.size(); i++) {
            if (i < 6)
            {
                start(i) = msg->data[i];
            }
            else
            {
                goal(i - 6) = msg->data[i];
            }
        }
        cmd_queue.push(std::make_tuple(start, goal));
    }
}

trajectory_msgs::JointTrajectory joint_trajectory(
    std::vector<Eigen::Matrix<double, 6, 1>> path, unsigned int seq)
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.seq = seq;
    for (size_t i = 0; i < path.size(); i++)
    {
        traj.joint_names.push_back(std::to_string(i));
        trajectory_msgs::JointTrajectoryPoint trajp;
        
        auto node = path.at(i);
        for (unsigned int qi = 0; qi < node.rows(); qi++)
        {
            trajp.positions.push_back(node(qi));
        }
        traj.points.push_back(trajp);
    }
    return traj;    
}


int main(int argc, char **argv){
    ros::init(argc, argv, "rrt");
    ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    auto n = ros::NodeHandle();
    auto rob = RRT::build_robot(n);

    // Build obstacles
    double sphere_r = 0.05;
    KDL::Frame init_frame;
    init_frame.p = KDL::Vector(0.22, 0, 0.6);
    std::vector<RRT::ICollidable*> obstacles;
    obstacles.push_back(new RRT::Sphere(init_frame, sphere_r));

    ros::Rate loop_rate(20);
    unsigned int seq = 0;
    ros::Publisher path_pub = n.advertise<trajectory_msgs::JointTrajectory>("path", 1);
    ros::Subscriber sub = n.subscribe<std_msgs::Float64MultiArray>("rrt_solver", 1, &solve_cmd);
    ROS_INFO("RRT Solver started");

    while(ros::ok())
    {
        
        ros::spinOnce();

        if (!cmd_queue.empty())
        {
            auto cmd = cmd_queue.front();
            cmd_queue.pop();
            auto start = std::get<0>(cmd);
            auto goal = std::get<1>(cmd);

            ROS_INFO_STREAM(
                "Received\n\tstart:\n\t" << start.transpose() 
                << "\n\tgoal:\n\t" << goal.transpose());

            const double spacing = 0.002;
            const double maxnorm = 0.1;
            const unsigned int try_merge_interval = 1000;
            const unsigned int maxiter = 100000;
            auto path = RRT::solver(
                rob, obstacles, start, goal, spacing, 
                maxnorm, try_merge_interval, maxiter);
            ROS_DEBUG("Solved path of %lu points", path.size());
            ROS_INFO_STREAM(
                "Full path:\n\tstart:\n\t" << path.front().transpose()
                << "\n\tgoal:\n\t" << path.back().transpose());
            
            path = path_reduction(path, rob, obstacles, spacing);
            ROS_DEBUG("Path redued size of %lu points", path.size());

            ROS_INFO_STREAM(
                "Reduced\n\tstart:\n\t" << path.front().transpose()
                << "\n\tgoal:\n\t" << path.back().transpose());
            
            auto traj = joint_trajectory(path, seq);
            seq++;
            path_pub.publish(traj);
            ROS_DEBUG("Solved path of %lu points", traj.points.size());
            ROS_INFO("path published");
        }

        loop_rate.sleep();
    }
    return 0;
}
