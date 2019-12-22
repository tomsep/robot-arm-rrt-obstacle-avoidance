#ifndef RRT_RRT_H
#define RRT_RRT_H

// from kdl packages
#include <vector>
#include <memory>
#include<kdl/jntarray.hpp>

namespace RRT
{


Eigen::Matrix<double, 6, 1> interpolate(
    Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to, double k);

Eigen::Matrix<double, 6, 1> clamp(
    Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to, double maxnorm);

double configuration_distance(Eigen::Matrix<double, 6, 1> from, Eigen::Matrix<double, 6, 1> to);

class Node
{

public:

    Eigen::Matrix<double, 6, 1> q_;
    Node* parent_;

    Node(Eigen::Matrix<double, 6, 1> q, Node* parent);
    void add_child(Node* child);
    size_t child_count() const;

private:
    
    
    std::vector<Node*> children;
    
};


class Tree
{

public:
    std::vector<Node*> nodes;

    Tree(Eigen::Matrix<double, 6, 1> q);
    ~Tree();
    Node* nearest_node(Eigen::Matrix<double, 6, 1> q);
    void add_node(Node* parent, Eigen::Matrix<double, 6, 1> q);
    bool merge(Tree tree, double limit);
    std::vector<Node*> path();
    Node* start;
    Node* end;

private:
    

    void apply_merge(Node* parent_tail, Node* child_tail);
};


}
#endif