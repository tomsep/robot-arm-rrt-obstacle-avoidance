/* ROS testing setup guide:
https://www.theconstructsim.com/ros-package-testing-part-1/
*/

#include "rrt_solver/rrt_solver.h"

#include <ros/ros.h>
#include <vector>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <Eigen/Core>


class MyTestSuite : public ::testing::Test {
  public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};


void assert_near_q(Eigen::Matrix<double, 6, 1> result, Eigen::Matrix<double, 6, 1> desired, double eps)
{
  ASSERT_EQ(result.rows() == desired.rows(), true);
  for (int i = 0; i < 6; i++)
  {
    ASSERT_NEAR(result(i), desired(i), eps);
  }
}


TEST_F(MyTestSuite, sphere_collision_checking)
{
  std::vector<RRT::ICollidable*> parts;
  std::vector<RRT::ICollidable*> obstacles;
  
  KDL::Frame frame;

  // part
  frame.p = KDL::Vector(0.1, 0, 0);
  auto sph1 = RRT::Sphere(frame, 0.1);
  parts.push_back(&sph1);

  // obstacle
  frame.p = KDL::Vector(-0.1, 0, 0);
  auto sph2 = RRT::Sphere(frame, 0.09);
  obstacles.push_back(&sph2);

  bool result = RRT::collision_check(parts, obstacles);
  ASSERT_EQ(result, false) << "Should not collide";
  
  // new obstacle that will collide touching the part
  frame.p = KDL::Vector(-0.1, 0, 0);
  auto sph3 = RRT::Sphere(frame, 0.1);
  obstacles.push_back(&sph3);

  result = RRT::collision_check(parts, obstacles);
  
  ASSERT_EQ(result, true) << "Should collide";
}


TEST_F(MyTestSuite, adapted_linspace)
{
  std::vector<double> values_d;
  double eps = 0.000001;
  Eigen::Matrix<double, 6, 1> q_from; 
  Eigen::Matrix<double, 6, 1> q_to;
  q_from << 0, 0.5, 0, 0, 0, -1;
  q_to << 0, 1, 0, 0, 0, 0.5;

  values_d = {-1.00000, -0.50000, 0.00000, 0.50000};
  auto result = RRT::adapted_relative_step_size(q_from, q_to, 0.5);
  
  ASSERT_NEAR(result, 0.3333333333333, eps);

  values_d = {-1.00000, -0.62500, -0.25000, 0.12500, 0.50000};
  result = RRT::adapted_relative_step_size(q_from, q_to, 0.4);

  ASSERT_NEAR(result, 0.25, eps);

  // 0 difference
  q_from << 0, 0.5, 0, 0, 0, -1;
  q_to << 0, 0.5, 0, 0, 0, -1;

  result = RRT::adapted_relative_step_size(q_from, q_to, 0.4);
  ASSERT_NEAR(result, 1, eps);
}


TEST_F(MyTestSuite, clamping)
{
  Eigen::Matrix<double, 6, 1> q_from, q_to, q_desired;
  double maxnorm;

  double eps = 0.001;
  q_from << 0, -1.5, 0, 0, 0, 0;
  q_to << 0, -0.5, 0, 0, 0, 0;
  q_desired << 0, -1, 0, 0, 0, 0;
  maxnorm = 0.5;

  auto clamped = RRT::clamp(q_from, q_to, maxnorm);
  assert_near_q(clamped, q_desired, eps);

  q_from << 0, -1.5, 2, 0, 0, 0;
  q_to << 0, -0.5, -0.5, 0, 0, 0;
  q_desired << 0 ,-1.31430, 1.53576, 0, 0, 0;

  clamped = RRT::clamp(q_from, q_to, maxnorm);
  assert_near_q(clamped, q_desired, eps);

  q_from << 0, -1.5, 2, 0, 0, 0;
  q_to = q_from;
  q_desired = q_from;

  clamped = RRT::clamp(q_from, q_to, maxnorm);
  assert_near_q(clamped, q_desired, eps);

}


TEST_F(MyTestSuite, tree_adding_nodes_and_merging)
{
  Eigen::Matrix<double, 6, 1> q_root, q, q_desired;
  double eps = 0.001;
  q_root << 0, 0, 0, 0, 0, 0;
  auto tree1 = std::make_shared<RRT::Tree>(q_root);

  q << 1, -2, 0, 0, 0, 0;
  auto nearest_node = tree1->nearest_node(q);
  q_desired = q_root;
  assert_near_q(nearest_node->q_, q_desired, eps);
  tree1->add_node(nearest_node, q);

  q << 0.5, -0.5, 0, 0, 0, 0;
  nearest_node = tree1->nearest_node(q);
  q_desired = q_root;
  assert_near_q(nearest_node->q_, q_desired, eps);
  tree1->add_node(nearest_node, q);

  q_desired = q;
  q << 0.25, -0.3, 0, 0, 0, 0;
  nearest_node = tree1->nearest_node(q);
  assert_near_q(nearest_node->q_, q_desired, eps);

  // Adding another identical q
  nearest_node = tree1->nearest_node(q_root);
  assert_near_q(nearest_node->q_, q_root, eps);

  // Test merging with another tree

  q << 2, -2, 0.1, 0, 0, 0;
  auto tree2 = std::make_shared<RRT::Tree>(q);

  q << 1, -2, 2, 0, 0, 0;
  nearest_node = tree2->nearest_node(q);
  tree2->add_node(nearest_node, q);

  bool merge_result = tree1->merge(tree2.get(), 0.01);
  ASSERT_EQ(merge_result, false);

  q << 1, -2.001, 0.001, 0, 0, 0;
  nearest_node = tree2->nearest_node(q);
  tree2->add_node(nearest_node, q);

  
  merge_result = tree1->merge(tree2.get(), 0.01);
  ASSERT_EQ(merge_result, true);


  auto path = tree1->path();
  assert_near_q(path.at(0)->q_, q_root, eps);

  q << 2, -2, 0.1, 0, 0, 0;
  assert_near_q(path.back()->q_, q, eps);
  
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
