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

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

const std::string PGK_ROOT_DIR = "/media/tomi/Data/documents/software/advanced_robotics_course/rrt_ws/src/rrt_solver/";


class MyTestSuite2 : public ::testing::Test {
  public:
    MyTestSuite2() {
    }
    ~MyTestSuite2() {}
};


void assert_near_q2(Eigen::Matrix<double, 6, 1> result, Eigen::Matrix<double, 6, 1> desired, double eps)
{
  ASSERT_EQ(result.rows() == desired.rows(), true);
  for (int i = 0; i < 6; i++)
  {
    ASSERT_NEAR(result(i), desired(i), eps);
  }
}

std::shared_ptr<RRT::RobotHull> build_test_robot(
  std::string root_link, std::vector<std::string> end_links)
{
    std::shared_ptr<RRT::RobotHull> rob = std::make_shared<RRT::RobotHull>();

    urdf::Model urdf;
    if (!urdf.initFile(PGK_ROOT_DIR + "tests/urdf/elfin_no_control_points.urdf"))
    {
        ROS_ERROR(
          "Failed to parse urdf file. "
          "Note: test urdf parsing will fail if PGK_ROOT_DIR is set incorrectly.");
    }
    
    double r = 0.02;
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf, tree);
    KDL::Frame init_frame;
    for (auto end_link : end_links)
    {
        //auto fk_solver = RRT::get_fk_solver(root_link, end_link, tree);
        KDL::Chain chain;
        if (!tree.getChain(root_link, end_link, chain))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_link << " --> " << end_link);
        }

        auto fk_solver =  KDL::ChainFkSolverPos_recursive(chain);
        RRT::ICollidable* shape = new RRT::Sphere(init_frame, r);
        rob->add_part(fk_solver, shape, chain.getNrOfJoints());
    }
    return rob;
}  



TEST_F(MyTestSuite2, collision_free_line)
{
  /*
  Test collision of elfin_link5. Its position is dependant on 5 joints so this test also
  ensures that not only end effector collision detection is possible.

  Start
  -----
  elfin_end_link at 
  (0.099695 0.0046828  0.59554) xyz and (90, 0, 90) rpy (world coord.)
  joints: 
  [-0.038542981394287246, 1.1327340534639134, 1.5898736950044032, 0.048026904546603966, 1.1167450420895788, 1.4268306326025728]

  End
  ---
  elfin_end_link at
  (0.5, 0, 0.6) xyz and (90, 0, 90) rpy
  joints:
  [-0.011869359246152733, -0.030880877807751617, 1.1021087454649434, 0.021360337268155227, 0.4388774263030708, 1.445796934865296]

  Obstacles:

  * Sphere r=0.05 at (0.22, -0.13, 0.6)
    elfin_link5 and elfin_end_link will NOT collide. (elfin_joint5 would however)
  * Sphere r=0.05 at (0.22, 0, 0.6)
    elfin_link5 and elfin_end_link will collide with
  */

  // Build test robot
  std::string root_link = "elfin_base_link";
  std::vector<std::string> end_links = {"elfin_link5", "elfin_end_link"};  //elfin_end_link
  std::shared_ptr<RRT::RobotHull> rob = build_test_robot(root_link, end_links);

  // Build obstacles
  double sphere_r = 0.05;
  KDL::Frame init_frame;
  init_frame.p = KDL::Vector(0.22, -0.13, 0.6);
  std::vector<RRT::ICollidable*> obstacles;
  obstacles.push_back(new RRT::Sphere(init_frame, sphere_r));

  Eigen::Matrix<double, 6, 1> q_from, q_to;
  q_from << -0.038542981394287246, 1.1327340534639134, 1.5898736950044032, 0.048026904546603966, 1.1167450420895788, 1.4268306326025728;
  q_to << -0.011869359246152733, -0.030880877807751617, 1.1021087454649434, 0.021360337268155227, 0.4388774263030708, 1.445796934865296;

  double spacing = 0.001;
  bool result;
  result = RRT::collision_free_line(q_from, q_to, spacing, rob, obstacles);

  ASSERT_EQ(result, true);

  init_frame.p = KDL::Vector(0.22, 0, 0.6);
  obstacles.push_back(new RRT::Sphere(init_frame, sphere_r));

  result = RRT::collision_free_line(q_from, q_to, spacing, rob, obstacles);

  ASSERT_EQ(result, false);

}


TEST_F(MyTestSuite2, solver_intergation_test)
{
  // Build test robot
  std::string root_link = "elfin_base_link";
  std::vector<std::string> end_links = {"elfin_link5"};
  std::shared_ptr<RRT::RobotHull> rob = build_test_robot(root_link, end_links);

  // Build obstacles
  double sphere_r = 0.05;
  KDL::Frame init_frame;
  init_frame.p = KDL::Vector(0.22, 0, 0.6);
  std::vector<RRT::ICollidable*> obstacles;
  obstacles.push_back(new RRT::Sphere(init_frame, sphere_r));

  Eigen::Matrix<double, 6, 1> start, goal;
  start << -0.038542981394287246, 1.1327340534639134, 1.5898736950044032, 0.048026904546603966, 1.1167450420895788, 1.4268306326025728;
  goal << -0.011869359246152733, -0.030880877807751617, 1.1021087454649434, 0.021360337268155227, 0.4388774263030708, 1.445796934865296;

  double spacing = 0.002;
  double maxnorm = 0.2;
  unsigned int try_merge_interval = 1000;
  unsigned int maxiter = 100000;
  std::shared_ptr<RRT::PointVisualizer> pviz_dummy;
  auto path = RRT::solver(rob, obstacles, start, goal, spacing, maxnorm, try_merge_interval, maxiter, pviz_dummy);

  ASSERT_EQ(path.size() >= 2, true);
}


TEST_F(MyTestSuite2, path_reduction)
{

  // Build test robot
  std::string root_link = "elfin_base_link";
  std::vector<std::string> end_links = {"elfin_link5"};
  std::shared_ptr<RRT::RobotHull> rob = build_test_robot(root_link, end_links);

  // Build obstacles
  double sphere_r = 0.05;
  KDL::Frame init_frame;
  init_frame.p = KDL::Vector(0.22, 0, 0.0);  // essentially no obstacle
  std::vector<RRT::ICollidable*> obstacles;
  obstacles.push_back(new RRT::Sphere(init_frame, sphere_r));

  std::vector<Eigen::Matrix<double, 6, 1>> path;
  Eigen::Matrix<double, 6, 1> q;

  q << 0, 0, 0, 0, 0, 0;
  path.push_back(q);

  q << 0.5, -0.25, 0, 0, 0, 0;
  path.push_back(q);

  q << 0, -0.25, -1, 0, 0, 0;
  path.push_back(q);

  q << 1, -0.5, 0, 0, 0, 0;
  path.push_back(q);

  double spacing = 0.01;
  auto reduced = RRT::path_reduction(path, rob, obstacles, spacing);

  ASSERT_EQ(reduced.size(), 2);

  double eps = 0.0001;

  q << 0, 0, 0, 0, 0, 0;
  assert_near_q2(reduced.front(), q, eps);

  q << 1, -0.5, 0, 0, 0, 0;
  assert_near_q2(reduced.back(), q, eps);

}