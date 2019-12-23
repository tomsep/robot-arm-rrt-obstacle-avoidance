#ifndef RRT_SOLVER_H
#define RRT_SOLVER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace RRT
{

class Sphere;

/* Interface for collidable shapes.

Visitor pattern for double dispatching to correct 
collision computation method for each possible shape combination.
*/
class ICollidable
{


    public:
        KDL::Frame frame;

        virtual ~ICollidable() {};
        virtual void update_pose(KDL::Frame f) {frame = f;}
        virtual bool collides(ICollidable& obj) = 0;
        virtual bool collides_with_sphere(Sphere& sphere) = 0;
        
};


class Sphere : public ICollidable
{
    public:
        double r;

        Sphere(KDL::Frame f, double radius) {frame = f; r = radius;}
        virtual bool collides(ICollidable& obj) {return obj.collides_with_sphere(*this);}
        virtual bool collides_with_sphere(Sphere& sphere);
    
};


class RobotHull
{
public:
    RobotHull(){}
    ~RobotHull();
    RobotHull(const RobotHull&) = delete;
    RobotHull&  operator=(const  RobotHull&)  = delete;
    std::vector<ICollidable*> collidables(Eigen::Matrix<double, 6, 1> q);
    void add_part(KDL::ChainFkSolverPos_recursive fk_solver, ICollidable* obj, unsigned int nm_joints);
    
    std::vector<ICollidable*> parts;
    std::vector<KDL::ChainFkSolverPos_recursive> fk_solvers;
    std::vector<std::unique_ptr<KDL::JntArray>> joint_arrs;
private:
    KDL::JntArray q_tmp;
    KDL::Frame frame_tmp;
    
};


class Node
{
public:
    Eigen::Matrix<double, 6, 1> q_;
    Node* parent_;

    Node(const Node&) = delete;
    Node&  operator=(const  Node&)  = delete;

    Node(Eigen::Matrix<double, 6, 1> q, Node* parent);
    void add_child(Node* child);
    size_t child_count() const;
    bool same_node(const Node& n);

private:
    std::vector<Node*> children;
};


class Tree
{
public:
    std::vector<Node*> nodes;

    Tree(Eigen::Matrix<double, 6, 1> q);
    ~Tree();
    Tree(const Tree&) = delete;
    Tree&  operator=(const  Tree&)  = delete;
    Node* nearest_node(Eigen::Matrix<double, 6, 1> q);
    void add_node(Node* parent, Eigen::Matrix<double, 6, 1> q);
    bool merge(Tree* tree, double limit);
    std::tuple<Node*, Node*> closest_nodepair_of_trees(Tree* tree);
    std::vector<Node*> path();
    Node* start;
    Node* end;
    void apply_merge(Node* parent_tail, Node* child_tail);
};


class PointVisualizer
{
public:
    PointVisualizer(
        std::shared_ptr<RRT::RobotHull> rob,
        std::string frame_id)
    {
        n_ = ros::NodeHandle();
        pub_ = n_.advertise<visualization_msgs::MarkerArray>("point_markers", 1, true);
        rob_ = rob;

        // Find any 6 joint part
        end_link_idx_ = -1;
        for (size_t i = 0; i < rob_->joint_arrs.size(); i++)
        {
            if ((*rob_->joint_arrs.at(i)).rows() == 6)
            {
                end_link_idx_ = i;
            }
        }
        if (end_link_idx_ == -1)
        {
            ROS_ERROR("No 6 joint robot parts. Will not attempt to visualize points.");
        }

        // Set default marker params
        marker_.header.frame_id = frame_id;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.pose.orientation.w = 1.0;

        marker_.color.r = 0.0f;
        marker_.color.g = 0.5f;
        marker_.color.b = 1.0f;
        marker_.color.a = 1.00;

    }

    void update(std::vector<RRT::Node*> nodes)
    {
        
        if (end_link_idx_ == 0)
        {
            return;
        }
        const double r = 0.005;
        KDL::JntArray q_tmp_(6);
        for (size_t i = 0; i < nodes.size(); i++)
        {
            marker_.header.stamp = ros::Time::now();
            marker_.id = i;
            q_tmp_.data = nodes.at(i)->q_;
            rob_->fk_solvers.at(end_link_idx_).JntToCart(q_tmp_, frame_tmp_);


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


private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    std::shared_ptr<RRT::RobotHull> rob_;
    visualization_msgs::MarkerArray marr_;
    visualization_msgs::Marker marker_;
    KDL::Frame frame_tmp_;
    KDL::JntArray q_;
    int end_link_idx_;
};

std::shared_ptr<RobotHull> build_robot(ros::NodeHandle nh);


bool collision_check(
    std::vector<ICollidable*> parts, 
    std::vector<ICollidable*> obstacles);


KDL::ChainFkSolverPos_recursive get_fk_solver(
    std::string root_link, 
    std::string end_link,
    KDL::Tree tree);


double adapted_relative_step_size(
    Eigen::Matrix<double, 6, 1> q_from, 
    Eigen::Matrix<double, 6, 1> q_to, 
    double spacing);


bool collision_free_line(
    Eigen::Matrix<double, 6, 1> q_from, 
    Eigen::Matrix<double, 6, 1> q_to,
    double spacing, std::shared_ptr<RobotHull> rob, std::vector<ICollidable*> obstacles);


Eigen::Matrix<double, 6, 1> clamp(
    Eigen::Matrix<double, 6, 1> from, 
    Eigen::Matrix<double, 6, 1> to, 
    double maxnorm);


double configuration_distance(Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to);


std::vector<Eigen::Matrix<double, 6, 1>> solver(
    std::shared_ptr<RobotHull> rob, 
    std::vector<ICollidable*> obstacles, 
    Eigen::Matrix<double, 6, 1> start,
    Eigen::Matrix<double, 6, 1> goal,
    double spacing, double maxnorm,
    unsigned int try_merge_interval, unsigned int maxiter,
    std::shared_ptr<RRT::PointVisualizer> viz);


std::vector<Eigen::Matrix<double, 6, 1>> path_reduction(
    std::vector<Eigen::Matrix<double, 6, 1>> path,
    std::shared_ptr<RobotHull> rob, 
    std::vector<ICollidable*> obstacles,
    double spacing);

}
#endif
