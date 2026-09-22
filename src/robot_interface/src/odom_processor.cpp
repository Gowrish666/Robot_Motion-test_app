#include "robot_interface/odom_processor.h"

#include <cmath>

namespace robot_interface
{
    OdomProcessor::OdomProcessor(
    double min_dt,
    double max_dt)
    : min_dt_(min_dt),
      max_dt_(max_dt),
      has_previous_sample_(false),
      previous_x_(0.0),
      previous_y_(0.0),
      previous_velocity_(0.0),
      current_velocity_(0.0),
      current_distance_(0.0),
      current_time_(0.0),
      current_acceleration_(0.0)
{
}
     void OdomProcessor::processOdom(
    const nav_msgs::Odometry::ConstPtr& msg)
{
    const ros::Time timestamp =
        msg->header.stamp.isZero()
            ? ros::Time::now()
            : msg->header.stamp;

    const double x =
        msg->pose.pose.position.x;

    const double y =
        msg->pose.pose.position.y;

    const double velocity_x =
        msg->twist.twist.linear.x;

    const double velocity_y =
        msg->twist.twist.linear.y;

    const double velocity =
        std::sqrt(
            velocity_x * velocity_x +
            velocity_y * velocity_y);

    if (!has_previous_sample_)
    {
        start_time_ = timestamp;
        previous_time_ = timestamp;

        previous_x_ = x;
        previous_y_ = y;

        previous_velocity_ = velocity;

        current_velocity_ = velocity;
        current_distance_ = 0.0;
        current_time_ = 0.0;
        current_acceleration_ = 0.0;

        samples_.clear();

        OdomSample sample;

        sample.time = current_time_;
        sample.velocity = current_velocity_;
        sample.distance = current_distance_;
        sample.acceleration = current_acceleration_;

        samples_.push_back(sample);

        has_previous_sample_ = true;

        return;
    }
    
    const double dt = (timestamp - previous_time_).toSec();

    if (dt < min_dt_)
    {
        return;
    }

        if (dt > max_dt_)
    {
        previous_time_ = timestamp;
        previous_x_ = x;
        previous_y_ = y;
        previous_velocity_ = velocity;

        return;
    }

    const double dx =
        x - previous_x_;

    const double dy =
        y - previous_y_;

    const double delta_distance =
        std::sqrt(
            dx * dx +
            dy * dy);

    current_distance_ += delta_distance;

    current_time_ =
        (timestamp - start_time_).toSec();

    current_velocity_ = velocity;

    current_acceleration_ =
        (current_velocity_ -
         previous_velocity_) /
        dt;

    OdomSample sample;

    sample.time = current_time_;
    sample.velocity = current_velocity_;
    sample.distance = current_distance_;
    sample.acceleration = current_acceleration_;

    samples_.push_back(sample);

    previous_time_ = timestamp;

    previous_x_ = x;
    previous_y_ = y;

    previous_velocity_ = current_velocity_;
}
    void OdomProcessor::reset()
{
    has_previous_sample_ = false;

    previous_x_ = 0.0;
    previous_y_ = 0.0;

    previous_velocity_ = 0.0;

    current_velocity_ = 0.0;
    current_distance_ = 0.0;
    current_time_ = 0.0;
    current_acceleration_ = 0.0;

    samples_.clear();

    start_time_ = ros::Time(0);
    previous_time_ = ros::Time(0);
}

bool OdomProcessor::hasData() const
{
    return has_previous_sample_;
}

const std::vector<OdomSample>&
OdomProcessor::samples() const
{
    return samples_;
}

double OdomProcessor::currentVelocity() const
{
    return current_velocity_;
}


double OdomProcessor::currentDistance() const
{
    return current_distance_;
}

double OdomProcessor::currentTime() const
{
    return current_time_;
}

double OdomProcessor::currentAcceleration() const
{
    return current_acceleration_;
}
}
