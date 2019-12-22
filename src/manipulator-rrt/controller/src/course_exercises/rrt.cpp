#include <kdl/kdl.hpp>
#include <utils/rrt.h>
#include <iostream>
#include <algorithm>


namespace RRT
{


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

    for (int i = 0; i < nodes.size(); i++)
    {
        double res = configuration_distance(nodes.at(i)->q_, q);//(nodes.at(i)->q_ - q).cwiseAbs().sum();
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


bool Tree::merge(Tree tree, double limit)
{
    std::vector<Node*>::iterator i;
    std::vector<Node*>::iterator i_other;

    for (i = nodes.begin();  i < nodes.end(); ++i)
    {

        for (i_other = tree.nodes.begin();  i_other < tree.nodes.end(); ++i_other)
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

Eigen::Matrix<double, 6, 1> clamp(
    Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to, double maxnorm)
{
    assert(maxnorm > 0);

    Eigen::Matrix<double, 6, 1> diff;
    Eigen::Matrix<double, 6, 1> clamped;
    diff = to - from;
    diff = (diff / (diff.norm() / maxnorm)).eval();
    clamped.noalias() = from + diff;
    return clamped;
}


Eigen::Matrix<double, 6, 1> interpolate(
    Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to, double k)
{
    assert(0 <= k <= 1);

    return from + (to - from) * k;;
}

double configuration_distance(Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to)
{
    return (to - from).cwiseAbs().sum();
}

}
