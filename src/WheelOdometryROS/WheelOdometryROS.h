#ifndef WHEELODOMETRYROS_H
#define WHEELODOMETRYROS_H

#include "../WheelOdom/WheelOdomFactory.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <wheel_odom/steeringAngle.h>
#include <wheel_odom/wheelSpeeds.h>

namespace wheel_odom
{

class WheelOdometryROS
{
    public:
    WheelOdometryROS( ros::NodeHandle nh, double length, double width );

    void steeringCallback( const wheel_odom::steeringAngleConstPtr& steeringAngle );

    void speedCallback( const wheel_odom::wheelSpeedsConstPtr& speed );

    private:
    WheelOdomPtr odom;
    WheelModel model_type;

    ros::Subscriber steering_sub;
    ros::Subscriber wheel_speeds_sub;
    ros::Publisher odom_pub;

    double m_dt;
    double m_timePrev;
    bool is_first_run;
};
}
#endif // WHEELODOMETRYROS_H
