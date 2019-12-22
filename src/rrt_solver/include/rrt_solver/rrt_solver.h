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
    unsigned int try_merge_interval, unsigned int maxiter);


std::vector<Eigen::Matrix<double, 6, 1>> path_reduction(
    std::vector<Eigen::Matrix<double, 6, 1>> path,
    std::shared_ptr<RobotHull> rob, 
    std::vector<ICollidable*> obstacles,
    double spacing);

}
#endif
