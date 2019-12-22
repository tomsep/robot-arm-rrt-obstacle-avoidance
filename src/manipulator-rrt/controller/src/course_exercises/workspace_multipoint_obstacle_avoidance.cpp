// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <queue>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>      // Jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainiksolverpos_lma.hpp>            // Inverse kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <utils/pseudo_inversion.h>         // J inv will segfault without this
#include <utils/helpers.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define NM_JOINTS 6


namespace arm_controllers
{

// Control point for obstacle avoidance
struct ControlPoint
{
    int nm_joints;
    double limit_distance;
    double k;
    KDL::Frame x_now;
    KDL::JntArray q_now;
    KDL::JntArray repl_vel;
    KDL::Jacobian J;
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;   
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver; 
};


class WorkspaceMultipointObstacleAvoidance : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/workspace_multipoint_obstacle_avoidance/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/workspace_multipoint_obstacle_avoidance/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/workspace_multipoint_obstacle_avoidance/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/workspace_multipoint_obstacle_avoidance/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        if (!n.getParam("root_link", root_link_name_))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_link_name_))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_link_name_, tip_link_name_, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_link_name_ << " --> " << tip_link_name_);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // Build ControlPoints
        std::vector<std::string> end_link_names = {"elfin_link3", "elfin_link4","elfin_link5", "elfin_link6", "elfin_end_link"};
        std::vector<double> limit_distances = {0.09, 0.09, 0.09, 0.09, 0.02};
        std::vector<double> k = {1, 1, 1, 1, 0.003};
        for (int i = 0; i < end_link_names.size(); i++)
        {
            control_points_.push_back(ControlPoint());
            
            KDL::Chain chain;

            if (!kdl_tree_.getChain(root_link_name_, end_link_names.at(i), chain))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain.");
                return false;
            }
            control_points_.at(i).nm_joints = chain.getNrOfJoints();
            control_points_.at(i).limit_distance = limit_distances.at(i);
            control_points_.at(i).k = k.at(i);
            control_points_.at(i).x_now = KDL::Frame();
            control_points_.at(i).q_now = KDL::JntArray(chain.getNrOfJoints());
            control_points_.at(i).repl_vel = KDL::JntArray(chain.getNrOfJoints());
            control_points_.at(i).J = KDL::Jacobian(chain.getNrOfJoints());
            control_points_.at(i).jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain)); 
            control_points_.at(i).fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
            
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));  // Jacobian solver
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));  // Forward kin solver
        ik_pos_lma_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        J_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);
        pub_xd_.reset(new helpers::PosePub("xd", n));
        pub_x_.reset(new helpers::PosePub("x", n));
        pub_obst_dists_.reset(new helpers::Array1DPub("obstacle_distances", n));
        pub_repl_vels_.reset(new helpers::Array1DPub("repel_vels", n));

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        sub_command_cart_ = n.subscribe<std_msgs::Float64MultiArray>("command_cart", 1, &WorkspaceMultipointObstacleAvoidance::cart_commandCB, this);
        // Push initial pose 
        std::array<double, NM_JOINTS> pose{0.1, 0, 0.6, 90, 0, 90};
        cart_queue_.push(pose);

        planner_.reset(new helpers::JointMotionPlan());
        potential_conf.k = 0.003;
        potential_conf.Q = 0.02;
        return true;
    }

    void cart_commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        } else
        {   
            std::array<double, NM_JOINTS> cmd;
            for (int i = 0; i < msg->data.size(); i++) {
                cmd[i] = msg->data[i];
            }
            cart_queue_.push(cmd);
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("WorkspaceMultipointObstacleAvoidance");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        if (planner_->finished())
        {

            if (!cart_queue_.empty())
            {   
                // Cartesian target converted to a joint target
                ROS_INFO("Planning new move");
                std::array<double, NM_JOINTS> xd_cmd = cart_queue_.front();
                cart_queue_.pop();

                xd_.p = KDL::Vector(xd_cmd[0], xd_cmd[1], xd_cmd[2]);
                xd_.M = KDL::Rotation::RPY(xd_cmd[3] * D2R, xd_cmd[4] * D2R, xd_cmd[5] * D2R);

                ik_pos_lma_solver_->CartToJnt(q_, xd_, qd_);
                std::cout << "qd_ ik solved: \n" << qd_.data << "\n\n";
                KDL::JntArray zero_jnt(6);
                
                // Target vel & acc = 0
                // TODO: replace current acc from zero to real value
                planner_->replan(q_, zero_jnt, zero_jnt, qd_, zero_jnt, zero_jnt, 4.0, dt);
            }
        }


        
        planner_->trajectory_point(qd_, qd_dot_, qd_ddot_);
        fk_pos_solver_->JntToCart(q_, x_);
        
        // Create obstacle as "table"
        KDL::Frame obst_frame(
            KDL::Rotation::RPY(0 * D2R, 0 * D2R, 0 * D2R),
            KDL::Vector(0.7, 0.0, 0.15)); // SWAP Elfin_Base X  = World Y
        helpers::RectObstacle obst = helpers::RectObstacle(obst_frame, KDL::Vector(0.45, 0.35, 0.15));

        Eigen::Matrix<double, -1, 1> obst_distances;
        obst_distances.resize(control_points_.size());
        // For each control point compute gradient for all joints up that link
        KDL::JntArray total_repl_vel(6);
        KDL::Vector repl_cart_grad;
        KDL::JntArray tmp_jntarr(6);
        for (int i = 0; i < control_points_.size(); i++)
        {   
            // Copy relevant joint values
            for (int j = 0; j < control_points_.at(i).nm_joints; j++)
            {
                control_points_.at(i).q_now(j) = q_(j);
            }
            // Solve x_now
            control_points_.at(i).fk_pos_solver->JntToCart(
                control_points_.at(i).q_now, control_points_.at(i).x_now);
            
            // Solve cartesian gradient
            potential_conf.Q = control_points_.at(i).limit_distance;  // use different cut-off dist
            potential_conf.k = control_points_.at(i).k;
            repl_cart_grad = helpers::repulsive_gradient(
                KDL::Vector(
                    control_points_.at(i).x_now.p[0], 
                    control_points_.at(i).x_now.p[1], 
                    control_points_.at(i).x_now.p[2]), 
                obst, potential_conf);
            
            // Save distance for plotting separately
            obst_distances(i) = obst.distance_to(
                KDL::Vector(
                control_points_.at(i).x_now.p[0], 
                control_points_.at(i).x_now.p[1], 
                control_points_.at(i).x_now.p[2]));

            // Gradient interpeted as velocity
            control_points_.at(i).jnt_to_jac_solver->JntToJac(
                control_points_.at(i).q_now, 
                control_points_.at(i).J);
            Eigen::MatrixXd J_inv_cpt;
            pseudo_inverse(control_points_.at(i).J.data, J_inv_cpt, false);

            tmp_jntarr(0) = repl_cart_grad[0];
            tmp_jntarr(1) = repl_cart_grad[1];
            tmp_jntarr(2) = repl_cart_grad[2];
            control_points_.at(i).repl_vel.data = J_inv_cpt * tmp_jntarr.data;

            // Add to total
            for (int j = 0; j < control_points_.at(i).nm_joints; j++)
            {
                total_repl_vel(j) += control_points_.at(i).repl_vel(j);
            }
        }

        pub_obst_dists_->publish(obst_distances);

        double tcp_dist = obst.distance_to(KDL::Vector(x_.p[0], x_.p[1], x_.p[2]));
        printf("TCP distance to obstacle: %f \n", tcp_dist);
        

        pub_repl_vels_->publish(total_repl_vel.data);

        // Gradient interpeted as velocity
        std::cout << "total_repl_vel:\n" << total_repl_vel.data << std::endl;
        qd_dot_(0) += total_repl_vel(0);
        qd_dot_(1) += total_repl_vel(1);
        qd_dot_(2) += total_repl_vel(2);
        qd_dot_(3) += total_repl_vel(3);
        qd_dot_(4) += total_repl_vel(4);
        qd_dot_(5) += total_repl_vel(5);

        
        pub_x_->publish(x_, root_link_name_);
        pub_xd_->publish(xd_, root_link_name_);
        
        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 

        // *** 2.3 Apply Torque Command to Actuator ***
        aux_d_.data = 
                M_.data * (
                Kp_.data.cwiseProduct(qd_.data - q_.data) + 
                Kd_.data.cwiseProduct(qd_dot_.data - qdot_.data) +
                qd_ddot_.data);
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

    }

    void stopping(const ros::Time &time)
    {
    }

  private:
    // others
    double t;
    helpers::PotentialConf potential_conf = helpers::PotentialConf();
    boost::scoped_ptr<helpers::JointMotionPlan> planner_;
    std::vector<ControlPoint> control_points_;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?
    std::string root_link_name_, tip_link_name_;

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;    // Jacobian solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_lma_solver_;

    // Jacobian
    KDL::Jacobian J_;
    Eigen::MatrixXd J_inv_;

    // Cartesian space
    KDL::Frame x_;
    KDL::Frame xd_;
    Eigen::Matrix<double, NM_JOINTS, 1> xd_dot_, xd_ddot_;
    Eigen::Matrix<double, NM_JOINTS, 1> ex_;  // cartesian error
    KDL::Twist ex_temp_;

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;
    boost::scoped_ptr<helpers::PosePub> pub_xd_;
    boost::scoped_ptr<helpers::PosePub> pub_x_;
    boost::scoped_ptr<helpers::Array1DPub> pub_obst_dists_;
    boost::scoped_ptr<helpers::Array1DPub> pub_repl_vels_;

    // ros subscriber
    ros::Subscriber sub_command_cart_;
    std::queue<std::array<double, NM_JOINTS>> cart_queue_;

    //pos_targets_(0) = 0.0;
    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::WorkspaceMultipointObstacleAvoidance, controller_interface::ControllerBase)