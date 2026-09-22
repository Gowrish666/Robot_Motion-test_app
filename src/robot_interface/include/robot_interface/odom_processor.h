#ifndef ROBOT_INTERFACE_ODOM_PROCESSOR_H
#define ROBOT_INTERFACE_ODOM_PROCESSOR_H

#include <nav_msgs/Odometry.h>
#include <vector>

namespace robot_interface 
{

    struct OdomSample
    {
         double time = 0.0;
         double velocity = 0.0;
         double distance = 0.0;
         double acceleration = 0.0;

    };


    class OdomProcessor
    {
        public:

            OdomProcessor(
                double min_dt = 0.001,
                double max_dt = 0.2
            );

            void processOdom(const nav_msgs::Odometry::ConstPtr& msg);

            void reset();

            bool hasData() const;

            const std::vector<OdomSample>& samples() const;

            double currentVelocity() const;
            double currentDistance() const;
            double currentTime() const;
            double currentAcceleration() const;

        private:
            double min_dt_;
            double max_dt_;

            bool has_previous_sample_;
            ros::Time start_time_;
            ros::Time previous_time_;

            double previous_x_;
            double previous_y_;
            
            double previous_velocity_;
            double current_velocity_;
            double current_distance_;
            double current_time_;
            double current_acceleration_;

            std::vector<OdomSample> samples_;
    };
}
#endif