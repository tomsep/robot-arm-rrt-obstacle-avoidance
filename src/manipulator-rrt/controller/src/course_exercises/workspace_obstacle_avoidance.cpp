// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <memory>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>      // Jacobian
#include <kdl/chainiksolverpos_lma.hpp>            // Inverse kinematics
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <queue>

#include <utils/pseudo_inversion.h>         // J inv will segfault without this
#include <utils/helpers.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define NM_JOINTS 6


namespace arm_controllers
{


class LinearJointMotion
{
public:
    LinearJointMotion(std::vector<Eigen::Matrix<double, 6, 1>> path, double vel)
    {
        vel_ = vel;
        path_ = path;
        q_now_ = path_.at(0);
        q_next_ = path_.at(1);
    }

    LinearJointMotion(trajectory_msgs::JointTrajectory trajectory, double vel)
    {
        
        Eigen::Matrix<double, 6, 1> q;
        for (auto trajp : trajectory.points)
        {
            
            for (size_t i = 0; i < trajp.positions.size(); i++)
            {
                q(i) = trajp.positions.at(i);
            }
            path_.push_back(q);
        }
        
        vel_ = vel;
        q_now_ = path_.at(0);
        q_next_ = path_.at(1);
    }

    Eigen::Matrix<double, 6, 1> next_target(double dt)
    {
        if (end_of_path_)
        {
            return path_.back();
        }

        auto segment_end = path_.at(interval_idx + 1);
        double max_mov = dt * vel_;

        auto max_diff_to_segment_end = (segment_end - q_now_).cwiseAbs().maxCoeff();

        if (max_diff_to_segment_end <= max_mov)
        {
            // Advance path if not at end
            // return segment end
            if (interval_idx + 2 < path_.size())
            {
                interval_idx++;
            }
            else
            {
                end_of_path_ = true;
            }
            q_now_ = segment_end;
            return segment_end;
        }
        else
        {
            // Compute intermediate pathpoint
            // Scale using max_mov
            double interpolate_factor = max_mov / max_diff_to_segment_end;
            auto joint_diff = segment_end - q_now_;
            auto q_inter = q_now_ + interpolate_factor * joint_diff;
            q_now_ = q_inter;
            return q_inter;
        }
    }
private:
    double vel_;
    size_t interval_idx = 0;
    Eigen::Matrix<double, 6, 1> q_now_, q_next_;
    std::vector<Eigen::Matrix<double, 6, 1>> path_;
    bool end_of_path_ = false;
};


class WorkspaceObstacleAvoidance : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
            if (n.getParam("/elfin/workspace_obstacle_avoidance/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/workspace_obstacle_avoidance/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/workspace_obstacle_avoidance/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/workspace_obstacle_avoidance/gains/elfin_joint" + si + "/pid/d", Kd[i]))
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


        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));  // Jacobian solver
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_lma_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        x_.data = Eigen::VectorXd::Zero(n_joints_);
        x_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        xd_.data = Eigen::VectorXd::Zero(n_joints_);
        xd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        xd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        x_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

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
        rrt_pub_ = n_public.advertise<std_msgs::Float64MultiArray>("rrt_solver", 1000);
        marker_pub_ = n_public.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        sub_command_jog_ = n.subscribe<std_msgs::Float64MultiArray>("command_jog", 1, &WorkspaceObstacleAvoidance::jog_commandCB, this);
        sub_command_cart_ = n.subscribe<std_msgs::Float64MultiArray>("command_cart", 1, &WorkspaceObstacleAvoidance::cart_commandCB, this);
        rrt_sub_ = n_public.subscribe<trajectory_msgs::JointTrajectory>("path", 10, &WorkspaceObstacleAvoidance::rrt_solved_CB, this);

        planner_.reset(new helpers::JointMotionPlan());
        lin_motion_ = nullptr;

        return true;
    }

    void jog_commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
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
            jog_queue_.push(cmd);
        }
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

    void rrt_solved_CB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
    {
        path_queue_.push(*msg);
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Task Space Controller");
    }

    void publish_trajectory(trajectory_msgs::JointTrajectory traj)
    {
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = 
            line_strip.header.frame_id = 
            line_list.header.frame_id = root_link_name_;
        points.header.stamp = line_strip.header.stamp = 
            line_list.header.stamp = ros::Time::now();
        points.action = line_strip.action = 
            line_list.action = visualization_msgs::Marker::ADD;

        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.004;
        points.scale.y = 0.004;
        points.color.g = 1.0f;
        points.color.a = 1.0;

        KDL::JntArray tmp_jntarr(6);
        KDL::Frame result;
        geometry_msgs::Point point;
        for (auto p : traj.points)
        {
            for (unsigned int i = 0; i < 6; i++)
            {
                tmp_jntarr(i) = p.positions.at(i);
            }
            fk_solver_->JntToCart(tmp_jntarr, result);
            point.x = result.p(0);
            point.y = result.p(1);
            point.z = result.p(2);
            points.points.push_back(point);
        }
        marker_pub_.publish(points);
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

        // ********* Current cart positions *******

        // x
        fk_solver_->JntToCart(q_, x_frame_);
        double r, p ,y;
        x_frame_.M.GetRPY(r, p, y);
        x_(0) = x_frame_.p(0);
        x_(1) = x_frame_.p(1);
        x_(2) = x_frame_.p(2);
        x_(3) = r;
        x_(4) = p;
        x_(5) = y;
        
        // x_dot
        jnt_to_jac_solver_->JntToJac(q_, J_);
        x_dot_.data = J_.data * qdot_.data;
        
        // x_ddot
        J_dot_.data = J_.data.derived();

        x_ddot_.data = J_dot_.data * qdot_.data;

        // ********
        if (current_trajectory_.points.empty())
        {
            if (!jog_queue_.empty()) 
            {   
                std::array<double, NM_JOINTS> qd_cmd = jog_queue_.front();
                jog_queue_.pop();
                for (int i = 0; i < qd_cmd.size(); i++)
                {
                    qd_(i) = qd_cmd[i] * D2R;
                }
                if (true)
                {
                    std_msgs::Float64MultiArray q_msg;
                    ROS_INFO_STREAM("" << q_.data.transpose() << "\n" << qd_.data.transpose());
                    for (unsigned int i = 0; i < 12; i++)
                    {
                        if (i < 6)
                        {
                            q_msg.data.push_back(q_(i));
                        }
                        else
                        {
                            q_msg.data.push_back(qd_(i - 6));
                        }
                    }
                    rrt_pub_.publish(q_msg);
                    ROS_INFO("Published path solve request");
                }
            }
            else if (!path_queue_.empty())
            {
                
                current_trajectory_ = path_queue_.front();
                path_queue_.pop();
                ROS_INFO("Received path of size %lu", current_trajectory_.points.size());
                publish_trajectory(current_trajectory_);
                lin_motion_.reset(new LinearJointMotion(current_trajectory_, max_vel_));
            }
        }


        if (planner_->finished())
        {
            if (!current_trajectory_.points.empty()) 
            {   
                ROS_INFO("Planning pathpoint move");
                auto trajpoint = current_trajectory_.points.front();
                //std::array<double, NM_JOINTS> qd_cmd = jog_queue_.front();
                //jog_queue_.pop();
                for (int i = 0; i < trajpoint.positions.size(); i++)
                {
                    qd_(i) = trajpoint.positions.at(i);
                    qd_dot_(i) = 0;
                    qd_ddot_(i) = 0;
                }
                ROS_INFO_STREAM("" << qd_.data.transpose());
                KDL::Frame frm;
                fk_solver_->JntToCart(qd_, frm);
                pub_xd_->publish(frm, root_link_name_);
                planner_->replan(q_, qd_dot_, qd_ddot_, qd_, qd_dot_, qd_ddot_, 2, dt);
                jogging_ = true;
                //current_trajectory_.points.pop_back();
                current_trajectory_.points.erase(current_trajectory_.points.begin());
            }
            else
            {
                // Stay in place
                for (int i = 0; i < 6; i++)
                {
                    qd_(i) = q_(i);
                    qd_dot_(i) = 0;
                    qd_ddot_(i) = 0;
                }
                planner_->replan(q_, qd_dot_, qd_ddot_, qd_, qd_dot_, qd_ddot_, 0.25, dt);
            }
            /*
            else if (!cart_queue_.empty())
            {   
                ROS_INFO("Planning cart move");
                std::array<double, NM_JOINTS> xd_cmd = cart_queue_.front();
                cart_queue_.pop();
                xd_(0) = xd_cmd[0];
                xd_(1) = xd_cmd[1];
                xd_(2) = xd_cmd[2];
                xd_(3) = xd_cmd[3] * D2R;
                xd_(4) = xd_cmd[4] * D2R;
                xd_(5) = xd_cmd[5] * D2R;

                xd_dot_(0) = 0;
                xd_dot_(1) = 0;
                xd_dot_(2) = 0;
                xd_dot_(3) = 0;
                xd_dot_(4) = 0;
                xd_dot_(5) = 0;

                xd_ddot_(0) = 0;
                xd_ddot_(1) = 0;
                xd_ddot_(2) = 0;
                xd_ddot_(3) = 0;
                xd_ddot_(4) = 0;
                xd_ddot_(5) = 0;
                
                planner_->replan(x_, x_dot_, x_ddot_, xd_, xd_dot_, xd_ddot_, 4.0, dt);
                jogging_ = false;
            }
            */
        }

        if (jogging_)
        {
            planner_->trajectory_point(qd_, qd_dot_, qd_ddot_);
            if (lin_motion_.get() != nullptr)
            {
                qd_.data = lin_motion_->next_target(dt);
            }
            else
            {
                ROS_ERROR("WAS NULLPTR");
            }
            
            id_solver_->JntToGravity(q_, G_); 
            e_.data = qd_.data - q_.data;
            tau_d_.data = G_.data + (Kp_.data.cwiseProduct(e_.data) - Kd_.data.cwiseProduct(qdot_.data));
        }
        else
        {   
            
            planner_->trajectory_point(xd_, xd_dot_, xd_ddot_);

            J_inv_ = J_.data.inverse();

            // *** 2.2 Compute model(M,C,G) ***
            id_solver_->JntToMass(q_, M_);
            id_solver_->JntToCoriolis(q_, qdot_, C_);
            id_solver_->JntToGravity(q_, G_); 

            // *** 2.3 Apply Torque Command to Actuator ***
            aux_d_.data = Kp_.data.cwiseProduct(xd_.data - x_.data) + Kd_.data.cwiseProduct(xd_dot_.data - x_dot_.data) + xd_ddot_.data - x_ddot_.data;
            tau_d_.data = M_.data * J_inv_ * aux_d_.data + C_.data + G_.data;

        }

        for (int i = 0; i < n_joints_; i++)
        {  
            joints_[i].setCommand(tau_d_(i));
        }

        // ********* 3. data 저장 *********
        //save_data();

        // ********* 4. state 출력 *********
        //print_state();

        
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(3);
        SaveData_[40] = e_dot_(4);
        SaveData_[41] = e_dot_(5);
        SaveData_[42] = e_dot_(6);

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 999)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            /*
            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");
            */

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");

            /*
            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");
            */

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    double max_vel_ = 0.1;  // rad/s
    ros::NodeHandle n_public;  // for non-private namespace
    bool jogging_ = true;
    boost::scoped_ptr<helpers::JointMotionPlan> planner_;
    trajectory_msgs::JointTrajectory current_trajectory_;
    std::unique_ptr<LinearJointMotion> lin_motion_;

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

    // Jacobian
    KDL::Jacobian J_;
    KDL::Jacobian J_dot_;
    Eigen::MatrixXd J_inv_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;    // Jacobian solver
    //boost::scoped_ptr<KDL::ChainIkSolverPos_NR> cart_to_joint_pos_;
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_lma_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
    //boost::scoped_ptr<KDL::ChainIkSolverVel_NR_JL> cart_to_joint_vel_;
    //boost::scoped_ptr<KDL::ChainIkSolverAcc_NR_JL> cart_to_joint_acc_;

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    // Cartesian space
    KDL::JntArray xd_;
    KDL::Frame x_frame_;
    KDL::JntArray x_;
    //Eigen::Matrix<double, NM_JOINTS, 1> xd_dot_, xd_ddot_;
    KDL::JntArray xd_ddot_, xd_dot_;
    KDL::JntArray x_dot_;
    KDL::JntArray x_ddot_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_, rrt_pub_;
    ros::Publisher pub_SaveData_, marker_pub_;
    boost::scoped_ptr<helpers::PosePub> pub_xd_;
    boost::scoped_ptr<helpers::PosePub> pub_x_;

    // ros subscriber
    ros::Subscriber sub_command_cart_, sub_command_jog_, rrt_sub_;
    std::queue<std::array<double, NM_JOINTS>> jog_queue_;
    std::queue<std::array<double, NM_JOINTS>> cart_queue_;
    std::queue<trajectory_msgs::JointTrajectory> path_queue_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::WorkspaceObstacleAvoidance, controller_interface::ControllerBase)