#ifndef HELPERS_HELPERS_H
#define HELPERS_HELPERS_H

#include <deque>

#include <map>
#include <kdl/kdl.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

namespace helpers
{
class Array1DPub
{
public:
    Array1DPub(const std::string topic, ros::NodeHandle &n, const uint32_t queue_size=1000) 
    {
        pub_ = n.advertise<std_msgs::Float64MultiArray>(topic, queue_size);
        
    }
    ~Array1DPub()
    {
        pub_.shutdown();
    }

    void publish(Eigen::Matrix<double, -1, 1> data)
    {   
        msg_.data.clear();
        for (int i = 0; i < data.rows(); i++)
        {
            msg_.data.push_back(data(i));
        }
        pub_.publish(msg_);
    }

private:
    ros::Publisher pub_;
    std_msgs::Float64MultiArray msg_;
};

class PosePub
{
public:
    PosePub(const std::string topic, ros::NodeHandle &n, const uint32_t queue_size=1000) 
    {
        pub_ = n.advertise<geometry_msgs::PoseStamped>(topic, queue_size);
        
    }
    ~PosePub()
    {
        pub_.shutdown();
    }

    void publish(KDL::Frame data, const std::string frame_id)
    {   

        header_.stamp = ros::Time::now();
        header_.frame_id = frame_id;

        point_.x = data.p.x();
        point_.y = data.p.y();
        point_.z = data.p.z();
        data.M.GetQuaternion(quaternion_.x, quaternion_.y, quaternion_.z, quaternion_.w);
        
        pose_.position = point_;
        pose_.orientation = quaternion_;

        pose_stamped_.header = header_;
        pose_stamped_.pose = pose_;

        pub_.publish(pose_stamped_);
    }
private:
    ros::Publisher pub_;
    geometry_msgs::PoseStamped pose_stamped_;
    geometry_msgs::Point point_;
    geometry_msgs::Pose pose_;
    std_msgs::Header header_;
    geometry_msgs::Quaternion quaternion_;
};


class JointMotionPlan
{
public:

    void replan(
        const KDL::JntArray q_start, const KDL::JntArray qdot_start, const KDL::JntArray qddot_start,
        const KDL::JntArray q_end, const KDL::JntArray qdot_end, const KDL::JntArray qddot_end,
        double duration, double dt)
    {   
        q_start_ = q_start;
        qdot_start_ = qdot_start;
        qddot_start_ = qddot_start;

        q_end_ = q_end;
        qdot_end_ = qdot_end;
        qddot_end_ = qddot_end;

        duration_ = duration;
        time_ = 0.0;
        dt_ = dt;
    }

    void replan_hold(
        const KDL::JntArray q_end, double dt)
    {   
        // Set duration to negative so that finished() always returns true
        duration_ = -1;
        time_ = 0.0;
        dt_ = dt;
        int nm_joints = q_end.rows();

        q_start_ = q_end;
        qdot_start_.data = Eigen::VectorXd::Zero(nm_joints);
        qddot_start_.data = Eigen::VectorXd::Zero(nm_joints);

        q_end_ = q_end;
        qdot_end_.data = Eigen::VectorXd::Zero(nm_joints);
        qddot_end_.data = Eigen::VectorXd::Zero(nm_joints);
        
    }

    void trajectory_point(
        KDL::JntArray &q_out, KDL::JntArray &qdot_out, KDL::JntArray &qddot_out)
    {   
        
        for (int i = 0; i < q_start_.rows(); i++)
        {   
            if (finished())
            {
                q_out(i) = q_end_(i);
                qdot_out(i) = qdot_end_(i);
                qddot_out(i) = qddot_end_(i);
            } 
            else
            {
                q_out(i) = trajectory_generator_pos(q_start_(i), q_end_(i), duration_);
                qdot_out(i) = trajectory_generator_vel(qdot_start_(i), qdot_end_(i), duration_);
                qddot_out(i) = trajectory_generator_acc(qddot_start_(i), qddot_end_(i), duration_);
            }
        }

        time_ = time_ + dt_;

    }

    bool finished()
    {
        if (time_ < duration_)
        {
            return false;
        } else
        {
            return true;
        }
    }

private:
    double time_ = 0;
    double dt_;
    KDL::JntArray q_start_, qdot_start_, qddot_start_;
    KDL::JntArray q_end_, qdot_end_, qddot_end_;
    double duration_ = -1;

    double trajectory_generator_pos(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
    }

    double trajectory_generator_vel(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
    }

    double trajectory_generator_acc(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
    }	
    

};

}
#endif