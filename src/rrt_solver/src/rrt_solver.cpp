#include "rrt_solver/rrt_solver.h"
#include <ros/ros.h>

#include <memory>
#include <tuple>
#include <vector>
#include <bits/stdc++.h> 

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

template <typename A, typename B>
void unzip(
    const std::vector<std::pair<A, B>> &zipped, 
    std::vector<A> &a, 
    std::vector<B> &b)
{
    for(size_t i=0; i<a.size(); i++)
    {
        a[i] = zipped[i].first;
        b[i] = zipped[i].second;
    }
}

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


std::vector<std::pair<Node*, Node*>> Tree::closest_nodepairs_of_trees(Tree* tree)
{
    using std::vector;
    using std::pair;

    vector<Node*>::iterator i;
    vector<Node*>::iterator i_other;
    //double shortest_distance = 1000000;
    //Node* this_node;
    //Node* other_node;

    vector<double> distances;
    distances.reserve(nodes.size() * tree->nodes.size());
    vector<pair<Node*, Node*>> nodepairs;
    nodepairs.reserve(nodes.size() * tree->nodes.size());


    for (i = nodes.begin();  i < nodes.end(); ++i)
    {

        for (i_other = tree->nodes.begin();  i_other < tree->nodes.end(); ++i_other)
        {
            double distance = configuration_distance((*i)->q_, (*i_other)->q_);
            distances.push_back(distance);
            nodepairs.push_back(std::make_pair(*i, *i_other));
            /*
            if (distance < shortest_distance)
            {
                shortest_distance = distance;
                this_node = *i;
                other_node = *i_other;
            }*/
            
        }
    }
    // Sort the vector of pairs
    vector<pair<double, pair<Node*, Node*>>> zipped;
    zip(distances, nodepairs, zipped);
    
    std::sort(std::begin(zipped), std::end(zipped), 
        [&](const auto& a, const auto& b)
    {
        return a.first < b.first;
    });

    unzip(zipped, distances, nodepairs);

    return nodepairs;
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


std::vector<Eigen::Matrix<double, 6, 1>> solver
(
    std::shared_ptr<RobotHull> rob, 
    std::vector<ICollidable*> obstacles, 
    Eigen::Matrix<double, 6, 1> start, 
    Eigen::Matrix<double, 6, 1> goal,
    double spacing, 
    unsigned int try_merge_interval, 
    unsigned int maxiter,
    std::shared_ptr<RRT::PointVisualizer> viz,
    std::vector<std::pair<int, double>> clamp_levels)
{
    auto tree1 = std::make_shared<RRT::Tree>(start);
    auto tree2 = std::make_shared<RRT::Tree>(goal);
    
    //size_t clamp_idx = 0;
    //double clamp_norm = clamps.at(clamp_idx);
    
    using std::vector;
    using std::pair;

    // TEMP
    //clamp_levels.push_back(std::make_pair(50, 2.0));
    //clamp_levels.push_back(std::make_pair(100, 1.0));
    //clamp_levels.push_back(std::make_pair(10000, 0.2));

    auto clamp_it = clamp_levels.begin();
    auto clamp_level = *clamp_it;
    ROS_INFO("Using clamp norm (%f) for %d iterations", clamp_level.second, clamp_level.first);

    for (unsigned int i = 0; i < maxiter; i++)
    {
        
        if (i > (unsigned)clamp_level.first && clamp_it != clamp_levels.end())
        {
            // Advance to next clamp level
            clamp_it++;
            if (clamp_it != clamp_levels.end())
            {
                clamp_level = *clamp_it;
                ROS_INFO("Using clamp norm (%f) for %d iterations", 
                    clamp_level.second, clamp_level.first);
            }
        }

        if (i % 50 == 0 && viz.get() != nullptr)
        {
            const double color1[3] = {0, 0.5, 1.0};
            viz->update(tree1->nodes, color1, "tree1");
            const double color2[3] = {0, 1.0, 0.5};
            viz->update(tree2->nodes, color2, "tree2");
            ros::spinOnce();
        }

        if (i % try_merge_interval == 0 && i != 0)
        {
            // Attempt merge
            ROS_INFO("Attempting to merge trees");
            auto closest = tree1->closest_nodepairs_of_trees(tree2.get());

            for (size_t ipair = 0; ipair < (closest.size() / 2); ipair++)
            {
                auto first = closest.at(ipair).first;
                auto second = closest.at(ipair).second;
                bool collisionfree = collision_free_line(
                    first->q_, second->q_, 
                    spacing, rob, obstacles);
                if (collisionfree)
                {
                    // Merge
                    ROS_INFO("Found collision free path");
                    tree1->apply_merge(first, second);

                    auto path = tree1->path();
                    std::vector<Eigen::Matrix<double, 6, 1>> joint_path;
                    for (auto path_node : path)
                    {
                        joint_path.push_back(path_node->q_);
                    }
                    return joint_path;
                }
            }
        }
        else
        {
            // Expand trees
            auto q_rand = Eigen::Matrix<double, 6, 1>::Random();

            auto nearest_node = tree1->nearest_node(q_rand);
            auto clamped = clamp(nearest_node->q_, q_rand, clamp_level.second);
            bool collisionfree = collision_free_line(
                nearest_node->q_, clamped, spacing, rob, obstacles);
            
            if (collisionfree)
            {
                tree1->add_node(nearest_node, clamped);
            }
            // Same for tree2 using same q_rand
            nearest_node = tree2->nearest_node(q_rand);
            clamped = clamp(nearest_node->q_, q_rand, clamp_level.second);
            collisionfree = collision_free_line(
                nearest_node->q_, clamped, spacing, rob, obstacles);
            
            if (collisionfree)
            {
                tree2->add_node(nearest_node, clamped);
            }
        }
        
    }
    ROS_ERROR("Failed to find a path!");
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
