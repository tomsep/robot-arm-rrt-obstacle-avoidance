#include "rrt_solver/rrt_solver.h"
#include <ros/ros.h>

#include <memory>
#include <tuple>
#include <vector>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>


namespace RRT
{

bool Sphere::collides_with_sphere(Sphere& sphere)
{
    auto distance = (sphere.frame.p - frame.p).Norm();
    if (distance <= sphere.r + r)
    {
        return true;
    }
    return false;
}


RobotHull::~RobotHull()
{
    for (auto part : parts)
    {
        delete part;
    }
    parts.clear();
}


std::vector<ICollidable*> RobotHull::collidables(Eigen::Matrix<double, 6, 1> q)
{
    for (size_t i = 0; i < parts.size(); i++)
    {   
        joint_arrs.at(i)->data = q.head(joint_arrs.at(i)->rows());
        fk_solvers.at(i).JntToCart(*joint_arrs.at(i), frame_tmp);
        parts.at(i)->update_pose(frame_tmp);
    }
    return parts;
}


void RobotHull::add_part(KDL::ChainFkSolverPos_recursive fk_solver, ICollidable* obj, unsigned int nm_joints)
{
    fk_solvers.push_back(fk_solver);
    parts.push_back(obj);
    auto jnt = std::make_unique<KDL::JntArray>(nm_joints);
    joint_arrs.push_back(std::move(jnt));
}


Node::Node(Eigen::Matrix<double, 6, 1> q, Node* parent)
{
    q_ = q;
    parent_ = parent;
}


void Node::add_child(Node* child)
{
    children.push_back(child);
}


size_t Node::child_count() const
{
    return children.size();
}


bool Node::same_node(const Node& n)
{
    if (&n == this)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}


Tree::Tree(Eigen::Matrix<double, 6, 1> q)
{
    start = new Node(q, nullptr);
    nodes.push_back(start);
}


Tree::~Tree()
{
    for (auto p : nodes)
    {
        delete p;
    }
    nodes.clear();
}


Node* Tree::nearest_node(Eigen::Matrix<double, 6, 1> q)
{
    double min_dist;
    Node* nearest;

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        double res = configuration_distance(nodes.at(i)->q_, q);
        if (res < min_dist || i == 0)
        {
            min_dist = res;
            nearest = nodes.at(i);
            
        }
    }
    return nearest;
}


void Tree::add_node(Node* parent, Eigen::Matrix<double, 6, 1> q)
{
    auto newnode = new Node(q, parent);
    nodes.push_back(newnode);
    parent->add_child(newnode);
}


bool Tree::merge(Tree* tree, double limit)
{
    std::vector<Node*>::iterator i;
    std::vector<Node*>::iterator i_other;

    for (i = nodes.begin();  i < nodes.end(); ++i)
    {

        for (i_other = tree->nodes.begin();  i_other < tree->nodes.end(); ++i_other)
        {
            double distance = configuration_distance((*i)->q_, (*i_other)->q_);
            if (distance <= limit)
            {
                // Merge possible
                apply_merge((*i), (*i_other));
                return true;
            } 
        }
    }

    return false;
}


void Tree::apply_merge(Node* parent_tail, Node* child_tail)
{
    Node* node_to_copy = child_tail;
    Node* parent_node = parent_tail;

    while (node_to_copy != nullptr)
    {
        add_node(parent_node, node_to_copy->q_);
        parent_node = nodes.back();
        node_to_copy = node_to_copy->parent_;
    }
    end = nodes.back();
}


std::tuple<Node*, Node*> Tree::closest_nodepair_of_trees(Tree* tree)
{
    std::vector<Node*>::iterator i;
    std::vector<Node*>::iterator i_other;
    double shortest_distance = 1000000;
    Node* this_node;
    Node* other_node;
    
    for (i = nodes.begin();  i < nodes.end(); ++i)
    {

        for (i_other = tree->nodes.begin();  i_other < tree->nodes.end(); ++i_other)
        {
            double distance = configuration_distance((*i)->q_, (*i_other)->q_);
            if (distance < shortest_distance)
            {
                shortest_distance = distance;
                this_node = *i;
                other_node = *i_other;
            } 
        }
    }
    return std::make_tuple(this_node, other_node);
}


std::vector<Node*> Tree::path()
{
    std::vector<Node*> pth;
    
    if (end != nullptr)
    {
        Node* parent = end->parent_;
        pth.push_back(end);
        while (parent != nullptr)
        {
            pth.push_back(parent);
            parent = parent->parent_;
        }
        std::reverse(pth.begin(), pth.end());
    }

    return pth;
}


KDL::ChainFkSolverPos_recursive get_fk_solver(
    std::string root_link, 
    std::string end_link,
    KDL::Tree tree)
{
    KDL::Chain chain;
    if (!tree.getChain(root_link, end_link, chain))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_link << " --> " << end_link);
    }

    return KDL::ChainFkSolverPos_recursive(chain);
}


bool collision_check(
    std::vector<ICollidable*> parts, 
    std::vector<ICollidable*> obstacles)
{
    bool result;
    for (auto& part : parts)
    {
        for (auto& obstacle : obstacles)
        {
            result = part->collides(*obstacle);
            if (result == true)
            {
                return true;
            }
        }
    }
    return result;
}


double adapted_relative_step_size(
    Eigen::Matrix<double, 6, 1> q_from, 
    Eigen::Matrix<double, 6, 1> q_to, 
    double spacing)
{   
    assert(spacing > 0);
    // result 0..1 relative step size
    // which will evenly interpolate q
    // with maximum absolute step of 'spacing'

    auto maxdif = (q_to - q_from).cwiseAbs().maxCoeff();

    int step_count = (int)(maxdif / spacing + 0.5);  // +0.5 to ceil
    if (step_count == 0)
    {
        return 1;
    }
    return 1 / (double)step_count;
}


std::shared_ptr<RobotHull> build_robot(ros::NodeHandle nh)
{
    auto rob = std::make_shared<RobotHull>();
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
    }

    std::string root_link;
    if (!nh.getParam("robot/root_link", root_link))
    {
        ROS_ERROR("Could not find root link name");
    }

    std::vector<std::string> end_links;
    if (!nh.getParam("robot/control_point_names", end_links))
    {
        ROS_ERROR("Could not find end links");
    }

    std::vector<double> radiuses;
    if (!nh.getParam("robot/control_point_radiuses", radiuses))
    {
        ROS_ERROR("Could not find radiuses");
    }

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf, tree);
    KDL::Frame init_frame;

    for (size_t i = 0; i < end_links.size(); i++)
    {
        KDL::Chain chain;
        if (!tree.getChain(root_link, end_links.at(i), chain))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_link << " --> " << end_links.at(i));
        }

        auto fk_solver =  KDL::ChainFkSolverPos_recursive(chain);
        ICollidable* shape = new Sphere(init_frame, radiuses.at(i));
        rob->add_part(fk_solver, shape, chain.getNrOfJoints());
    }
    return rob;
}   


bool collision_free_line(
    Eigen::Matrix<double, 6, 1> q_from, 
    Eigen::Matrix<double, 6, 1> q_to,
    double spacing, 
    std::shared_ptr<RobotHull> rob,
    std::vector<ICollidable*> obstacles)
{
    // TODO: Try last first for possible quick elimination, reverse?
    double stepsize = adapted_relative_step_size(q_from, q_to, spacing);
    double k = stepsize;  // 0 assumed already free
    auto q_dif = q_to - q_from;

    while (k <= 1)
    {   
        auto q = q_from + q_dif * k;
        auto parts = rob->collidables(q);
        bool collision = collision_check(parts, obstacles);
        if (collision)
        {
            return false;
        }
        k += stepsize;
    }
    return true;
}


Eigen::Matrix<double, 6, 1> clamp(
    Eigen::Matrix<double, 6, 1> from,
    Eigen::Matrix<double, 6, 1> to,
    double maxnorm)
{
    assert(maxnorm > 0);

    Eigen::Matrix<double, 6, 1> diff;
    diff = to - from;
    diff = (diff / (diff.norm() / maxnorm)).eval();
    if (diff.hasNaN())
    {
        return to;
    }
    else
    {
        return from + diff;
    }
}


double configuration_distance(Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to)
{
    return (to - from).cwiseAbs().sum();
}


std::vector<Eigen::Matrix<double, 6, 1>> solver(std::shared_ptr<RobotHull> rob, std::vector<ICollidable*> obstacles, 
                    Eigen::Matrix<double, 6, 1> start, Eigen::Matrix<double, 6, 1> goal,
                    double spacing, double maxnorm, unsigned int try_merge_interval, unsigned int maxiter)
{
    auto tree1 = std::make_shared<RRT::Tree>(start);
    auto tree2 = std::make_shared<RRT::Tree>(goal);

    for (unsigned int i = 0; i < maxiter; i++)
    {
        if (i % try_merge_interval == 0 && i != 0)
        {
            // TRY merge
            auto closest = tree1->closest_nodepair_of_trees(tree2.get());

            bool collisionfree = collision_free_line(
                std::get<0>(closest)->q_, std::get<1>(closest)->q_, spacing, rob, obstacles);
            if (collisionfree)
            {
                // Merge
                tree1->apply_merge(std::get<0>(closest), std::get<1>(closest));

                auto path = tree1->path();
                std::vector<Eigen::Matrix<double, 6, 1>> joint_path;
                for (auto path_node : path)
                {
                    joint_path.push_back(path_node->q_);
                }
                return joint_path;
            }
        }
        else
        {
            // Expand trees
            auto q_rand = Eigen::Matrix<double, 6, 1>::Random();

            auto nearest_node = tree1->nearest_node(q_rand);
            auto clamped = clamp(nearest_node->q_, q_rand, maxnorm);
            bool collisionfree = collision_free_line(
                nearest_node->q_, clamped, spacing, rob, obstacles);
            
            if (collisionfree)
            {
                tree1->add_node(nearest_node, clamped);
            }
            // Same for tree2 using same q_rand
            nearest_node = tree2->nearest_node(q_rand);
            clamped = clamp(nearest_node->q_, q_rand, maxnorm);
            collisionfree = collision_free_line(
                nearest_node->q_, clamped, spacing, rob, obstacles);
            
            if (collisionfree)
            {
                tree2->add_node(nearest_node, clamped);
            }
        }
        
    }
    ROS_ERROR("Failed to find path");
    return std::vector<Eigen::Matrix<double, 6, 1>>();

} 


std::vector<Eigen::Matrix<double, 6, 1>> path_reduction(
    std::vector<Eigen::Matrix<double, 6, 1>> path,
    std::shared_ptr<RobotHull> rob, 
    std::vector<ICollidable*> obstacles,
    double spacing)
{
    std::vector<Eigen::Matrix<double, 6, 1>> reduced;
    reduced.push_back(path.front());
    Eigen::Matrix<double, 6, 1> a = path.front();
    while (a != path.back())
    {
        for (std::vector<Eigen::Matrix<double, 6, 1>>::reverse_iterator i = path.rbegin();
            i != path.rend(); ++i)
        {
            bool collisionfree = collision_free_line(
                a, (*i), spacing, rob, obstacles);
            
            if (a == (*i))
            {
                break;
            }
            else if (collisionfree)
            {
                reduced.push_back(*i);
                a = (*i);
                break;
                
            }
        } 
    }
    return reduced;
}
}
