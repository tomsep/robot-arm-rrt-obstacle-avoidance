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

class LowPass
{
public:

    LowPass(int buffer_len)
    {
        buffer_len_ = buffer_len;
    }
    
    KDL::JntArray filter(KDL::JntArray arr)
    {
        buffer_.push_back(arr);
        if (buffer_.size() > buffer_len_)
        {
            buffer_.pop_front();
        }

        // Zero result
        result_(0) = 0;
        result_(1) = 0;
        result_(2) = 0;
        result_(3) = 0;
        result_(4) = 0;
        result_(5) = 0;

        // Summing
        for (int i = 0; i < buffer_.size(); i++)
        {
            for (int j = 0; j < 6; j++)
            {   
                result_(j) = result_(j) + buffer_.at(i)(j);
            }
        }

        // Averaging
        for (int i = 0; i < 6; i++)
        {   
            result_(i) = result_(i) / double(buffer_.size());
        }

        return result_;
    }
private:
    std::deque<KDL::JntArray> buffer_;
    KDL::JntArray result_ = KDL::JntArray(6);
    int buffer_len_;
};



class RectObstacle
{
// Oriented Bounding Box (OBB)
// See: https://wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/

public:
    RectObstacle(KDL::Frame frame, KDL::Vector dims)
    {
        frame_ = frame;
        dims_ = dims;  // side vertices w.r.t. center of the box
    }
    double distance_to(KDL::Vector point)
    {   
        // 1. Transfrom point to obstacle's space
        auto R = Eigen::Matrix<double, 3, 3>(frame_.M.data);
        R.transposeInPlace();

        // Type conversion
        auto difference = (point - frame_.p);

        Eigen::Matrix<double, 3, 3> eigen_vec_diff;
        eigen_vec_diff(0) = difference[0];
        eigen_vec_diff(1) = difference[1];
        eigen_vec_diff(2) = difference[2];

        auto point_o_eigen_vec = R * eigen_vec_diff;
        KDL::Vector point_o = KDL::Vector(point_o_eigen_vec(0), point_o_eigen_vec(1), point_o_eigen_vec(2));
        // 2. Point closest on the rectangle surface
        KDL::Vector point_on_rect = clamp(point_o);

        // 3. Distance from rect surface to point
        double dist = (point_o - point_on_rect).Norm();
        return dist;
    }


private:
    KDL::Frame frame_;
    KDL::Vector dims_;

    KDL::Vector clamp(KDL::Vector vec)
    {   
        KDL::Vector result;
        for (int i = 0; i < 3; i++)
        {
            if (vec[i] > dims_[i])
            {
                result[i] = dims_[i];
            }
            else if (vec[i] < -dims_[i])
            {
                result[i] = -dims_[i];
            }
            else
            {
                result[i] = vec[i];
            }
        }

        return result;
    }
};

class PointObstacle
{
public:
    PointObstacle(KDL::Vector point)
    {
        point_ = point;
    }

    double distance_to(KDL::Vector point)
    {
        return (point - point_).Norm();
    }

private:
    KDL::Vector point_;

};

class ReactLimit
{
// Oriented Bounding Box (OBB)
// See: https://wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/
// MIUKK

public:
    ReactLimit(KDL::JntArray arr)
    {
        arr_ = arr;
       // dims_ = dims;  // side vertices w.r.t. center of the box
    }
    double distance_to(KDL::Vector point)
    {   
        /* 1. Transfrom point to obstacle's space
        auto R = Eigen::Matrix<double, 3, 3>(frame_.M.data);
        R.transposeInPlace();

        // Type conversion
        auto difference = (point - frame_.p);

        Eigen::Matrix<double, 3, 3> eigen_vec_diff;
        eigen_vec_diff(0) = difference[0];
        eigen_vec_diff(1) = difference[1];
        eigen_vec_diff(2) = difference[2];

        auto point_o_eigen_vec = R * eigen_vec_diff;
        KDL::Vector point_o = KDL::Vector(point_o_eigen_vec(0), point_o_eigen_vec(1), point_o_eigen_vec(2));
        // 2. Point closest on the rectangle surface
        KDL::Vector point_on_rect = clamp(point_o);

        // 3. Distance from rect surface to point
        double dist = (point_o - point_on_rect).Norm();
        */
        //return dist;
    }
private:
    KDL::JntArray arr_ ;
    
};


struct PotentialConf
{
    double k = 0.75;
    double Q = 0.1;
    double delta = 0.0001;
};


inline KDL::Vector distance_gradient(KDL::Vector point, RectObstacle obstacle, double delta)
{
    // Computes gradient (d(D)/d(axis)) near 'point' using central difference with step size 'delta'.
    auto result = KDL::Vector();

    for (int i = 0; i < 3; i++)
    {   
        auto delta_point = KDL::Vector(point);
        delta_point[i] = delta_point[i] + delta;
        double dist_plus = obstacle.distance_to(delta_point);

        delta_point = KDL::Vector(point);
        delta_point[i] = delta_point[i] - delta;
        double dist_minus = obstacle.distance_to(delta_point);

        result[i] = (dist_plus - dist_minus) / (2*delta);
    }

    return result;
}

inline KDL::Vector repulsive_gradient(KDL::Vector point, RectObstacle obstacle, PotentialConf conf)
{   
        // Computes repulsive potential gradient near 'point'.
        KDL::Vector grad = distance_gradient(point, obstacle, conf.delta);

        auto result = KDL::Vector(0, 0, 0);

        // Repulsive
        double D = obstacle.distance_to(point);

        // Undefined behaviour if D == 0
        if (D <= conf.Q)
        {
            for (int i = 0; i < 3; i++)
            {
                result[i] = conf.k * (1/D - 1/conf.Q) * (1/std::pow(D, 2) * grad[i]);
            }
        }

        return result;
}


inline double joint_limit_gradient(double limit, double angle_now, PotentialConf conf)
{   
    // Symmetric limit (+/- limit)
    // Assumes that angle_now never goes over limit
    
    // Distance to limit
    double D;
    if (angle_now < 0)
    {
        D = -(angle_now + limit);
    }
    else
    {
        D = limit - angle_now;
    }


    // If under cut-off distance apply gradient
    double gradient;
    if (D <= conf.Q)
    {   
        // dD/dangle derivative always 1.                                              v
        //                                                                             v
        gradient = conf.k * (1/std::abs(D) - 1/conf.Q) * (1/std::pow(std::abs(D), 2) * 1);
    }
    else
    {
        gradient = 0;
    }

    // Apply correct gradient sign
    if (D < 0)
    {
        return gradient;  // positive joint vel
    }
    else
    {
        return gradient * -1;  // negative joint vel
    }
}

}
#endif