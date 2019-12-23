#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "rrt_solver/rrt_solver.h"
#include <Eigen/Core>
#include <vector>
#include <queue>
#include <tuple>
#include<string>
ros::Publisher marker_pub;

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

std::vector<RRT::ICollidable*> build_obstacles(ros::NodeHandle n)
{
    using std::vector;
    vector<double> x, y, z, r;
    vector<RRT::ICollidable*> obstacles;

    if (!n.getParam("obstacles/x", x))
    {
        ROS_ERROR("Could not find obstacle x param");
    }
    if (!n.getParam("obstacles/y", y))
    {
        ROS_ERROR("Could not find obstacle y param");
    }
    if (!n.getParam("obstacles/z", z))
    {
        ROS_ERROR("Could not find obstacle z param");
    }
    if (!n.getParam("obstacles/r", r))
    {
        ROS_ERROR("Could not find obstacle r param");
    }
    size_t size = x.size();
    if (y.size() != size || z.size() != size || r.size() != size)
    {
        ROS_ERROR("Obstacle's parameter dimensions mismatch");
    }

    KDL::Frame init_frame;

    for (size_t i = 0; i < x.size(); i++)
    {
        init_frame.p = KDL::Vector(x.at(i), y.at(i), z.at(i));
        obstacles.push_back(new RRT::Sphere(init_frame, r.at(i)));

    }
    return obstacles;
}


class ObstacleVisualizer
{
public:
    ObstacleVisualizer(
        std::vector<RRT::ICollidable*> obstacles,
        std::string frame_id)
    {
        obstacles_ = obstacles;
        n_ = ros::NodeHandle();
        pub_ = n_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1, true);

        part_count_ = obstacles.size();
        marr_.markers.reserve(part_count_);

        // Set default marker params
        marker_.header.frame_id = frame_id;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.pose.orientation.w = 1.0;

        marker_.color.r = 0.2f;
        marker_.color.g = 1.0f;
        marker_.color.b = 0.2f;
        marker_.color.a = 1.0;

    }
    void update()
    {
        for (size_t i = 0; i < part_count_; i++)
        {
            marker_.header.stamp = ros::Time::now();
            marker_.id = i;

            auto sphere = (RRT::Sphere*)obstacles_.at(i);
            double r = sphere->r;
            auto frame = sphere->frame;

            marker_.scale.x = r * 2;
            marker_.scale.y = r * 2;
            marker_.scale.z = r * 2;
            marker_.pose.position.x = frame.p(0);
            marker_.pose.position.y = frame.p(1);
            marker_.pose.position.z = frame.p(2);

            marr_.markers.push_back(marker_);
        }
        pub_.publish(marr_);
        marr_.markers.clear();
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    std::vector<RRT::ICollidable*> obstacles_;
    visualization_msgs::MarkerArray marr_;
    visualization_msgs::Marker marker_;
    size_t part_count_;
};


class HullVisualizer
{
public:
    HullVisualizer(
        std::shared_ptr<RRT::RobotHull> rob,
        std::string frame_id)
    {
        rob_ = rob;
        n_ = ros::NodeHandle();
        pub_ = n_.advertise<visualization_msgs::MarkerArray>("robothull_markers", 1);
        sub_ = n_.subscribe<sensor_msgs::JointState>("joint_states", 10, &HullVisualizer::joint_callback, this);

        part_count_ = rob->parts.size();
        marr_.markers.reserve(part_count_);
        q_.resize(6);

        for (size_t i = 0; i < rob_->joint_arrs.size(); i++)
        {
            q_tmps_.push_back(*(rob_->joint_arrs.at(i)));
        }

        // Set default marker params
        marker_.header.frame_id = frame_id;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.pose.orientation.w = 1.0;

        marker_.color.r = 0.0f;
        marker_.color.g = 1.0f;
        marker_.color.b = 0.0f;
        marker_.color.a = 0.33;

    }
    void update()
    {
        for (size_t i = 0; i < part_count_; i++)
        {
            marker_.header.stamp = ros::Time::now();
            marker_.id = i;
            q_tmps_.at(i).data = q_.data.head(q_tmps_.at(i).rows());
            rob_->fk_solvers.at(i).JntToCart(q_tmps_.at(i), frame_tmp_);
            double r = ((RRT::Sphere*)rob_->parts.at(i))->r;
            marker_.scale.x = r * 2;
            marker_.scale.y = r * 2;
            marker_.scale.z = r * 2;
            marker_.pose.position.x = frame_tmp_.p(0);
            marker_.pose.position.y = frame_tmp_.p(1);
            marker_.pose.position.z = frame_tmp_.p(2);
            marr_.markers.push_back(marker_);
        }
        pub_.publish(marr_);
        marr_.markers.clear();
    }

    void joint_callback(const sensor_msgs::JointStateConstPtr &msg)
    {
        auto posvec = msg->position;
        for (size_t i = 0; i < posvec.size(); i++)
        {
            q_(i) = posvec.at(i);
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    visualization_msgs::MarkerArray marr_;
    std::shared_ptr<RRT::RobotHull> rob_;
    visualization_msgs::Marker marker_;
    size_t part_count_;
    std::vector<KDL::JntArray> q_tmps_;
    KDL::Frame frame_tmp_;
    KDL::JntArray q_;
};





int main(int argc, char **argv){
    ros::init(argc, argv, "rrt");
    ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    auto n = ros::NodeHandle();
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100, true);

    
    // Load solver params
    double spacing;
    std::vector<double> clamp_norms;
    std::vector<double> clamps;
    std::vector<int> clamps_iters;
    int try_merge_interval, maxiter;

    if (!n.getParam("solver/spacing", spacing))
    {
        ROS_ERROR("Could not find spacing param");
    }

    if (!n.getParam("solver/clamping/norm", clamp_norms))
    {
        ROS_ERROR("Could not find clamping norm params");
    }

    if (!n.getParam("solver/clamping/iter", clamps_iters))
    {
        ROS_ERROR("Could not find clamping norm params");
    }

    std::vector<std::pair<int, double>> clamp_levels;
    RRT::zip(clamps_iters, clamp_norms, clamp_levels);

    if (!n.getParam("solver/merge_attempt_interval", try_merge_interval))
    {
        ROS_ERROR("Could not find merge_attempt_interval param");
    }

    if (!n.getParam("solver/maxiter", maxiter))
    {
        ROS_ERROR("Could not find maxiter param");
    }
    
    auto rob = RRT::build_robot(n);
    auto hull_viz = HullVisualizer(rob, "world");

    auto obstacles = build_obstacles(n);
    auto obst_viz = ObstacleVisualizer(obstacles, "world");

    auto point_viz = std::make_shared<RRT::PointVisualizer>(rob, "world");
    
    ros::Rate loop_rate(30);
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
                "Received command\n\tstart:\n\t" << start.transpose() 
                << "\n\tgoal:\n\t" << goal.transpose());

            auto path = RRT::solver(
                rob, obstacles, start, goal, spacing, 
                try_merge_interval, maxiter, point_viz, clamp_levels);
        
            path = RRT::path_reduction(path, rob, obstacles, spacing);
            ROS_INFO("Solved path with %lu via points", path.size());
            auto traj = joint_trajectory(path, seq);
            
            seq++;
            path_pub.publish(traj);
        }

        hull_viz.update();
        obst_viz.update();
        loop_rate.sleep();
    }
    return 0;
}
