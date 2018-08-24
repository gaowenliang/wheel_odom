#include "WheelOdometryROS.h"

wheel_odom::WheelOdometryROS::WheelOdometryROS( ros::NodeHandle nh, double length, double width )
: is_first_run( true )
{
    model_type = OMNI_WHEEL;
    odom = wheel_odom::WheelOdomFactory::newWheelOdom( )->init( model_type, length, width );

    steering_sub
    = nh.subscribe< wheel_odom::steeringAngle >( "steering", //
                                                 1,
                                                 &WheelOdometryROS::steeringCallback,
                                                 this,
                                                 ros::TransportHints( ).tcpNoDelay( true ) );

    wheel_speeds_sub
    = nh.subscribe< wheel_odom::wheelSpeeds >( "wheelSpeeds", //
                                               1,
                                               &WheelOdometryROS::speedCallback,
                                               this,
                                               ros::TransportHints( ).tcpNoDelay( true ) );

    odom_pub = nh.advertise< nav_msgs::Odometry >( "wheel_odom", 1 );
}

void
wheel_odom::WheelOdometryROS::steeringCallback( const wheel_odom::steeringAngleConstPtr& steeringAngle )
{
    odom->setsteeringAngle( steeringAngle->steering );
}

void
wheel_odom::WheelOdometryROS::speedCallback( const wheel_odom::wheelSpeedsConstPtr& speed )
{
    if ( is_first_run )
    {
        is_first_run = false;
        m_timePrev   = speed->header.stamp.toSec( );
        return;
    }
    else
    {
        m_dt       = speed->header.stamp.toSec( ) - m_timePrev;
        m_timePrev = speed->header.stamp.toSec( );
    }

    switch ( model_type )
    {
        case FRONT_WHEEL:
            odom->setSpeedIndex( speed->speedRF, 0 );
            odom->setSpeedIndex( speed->speedLF, 1 );
            break;
        case REAR_WHEEL:
            odom->setSpeedIndex( speed->speedRB, 0 );
            odom->setSpeedIndex( speed->speedLB, 1 );
            break;
        case TRICYCLE:
            odom->setSpeedIndex( ( speed->speedLF + speed->speedRF ) / 2, 0 );
            break;
        case OMNI_WHEEL:
            odom->setSpeedIndex( speed->speedRF, 0 );
            odom->setSpeedIndex( speed->speedLF, 1 );
            odom->setSpeedIndex( speed->speedLB, 2 );
            break;
        case MECANUM_WHEEL:
            odom->setSpeedIndex( speed->speedRF, 0 );
            odom->setSpeedIndex( speed->speedLF, 1 );
            odom->setSpeedIndex( speed->speedLB, 2 );
            odom->setSpeedIndex( speed->speedRB, 3 );
            break;
        default:
            break;
    }

    odom->setDt( m_dt );

    odom->calcOdom( );

    Pose2Dd odom2d = odom->getPose( );
    Pose2Dd vel2d  = odom->getVel( );

    {
        double yaw = odom2d.yaw;

        nav_msgs::OdometryPtr odom_msg( new nav_msgs::Odometry );
        odom_msg->header.stamp    = speed->header.stamp;
        odom_msg->header.frame_id = "world";
        odom_msg->child_frame_id  = "base_link";

        odom_msg->pose.pose.position.x = odom2d.x;
        odom_msg->pose.pose.position.y = odom2d.y;
        odom_msg->pose.pose.position.z = 0;

        odom_msg->pose.pose.orientation.w = cos( yaw / 2 );
        odom_msg->pose.pose.orientation.x = 0.0;
        odom_msg->pose.pose.orientation.y = 0.0;
        odom_msg->pose.pose.orientation.z = sin( yaw / 2 );

        odom_msg->twist.twist.linear.x = vel2d.x;
        odom_msg->twist.twist.linear.y = 0; // vel2d.y; // assume velocity y is 0
        odom_msg->twist.twist.linear.z = 0;

        odom_msg->twist.twist.angular.x = 0;
        odom_msg->twist.twist.angular.y = 0;
        odom_msg->twist.twist.angular.z = vel2d.yaw;
        odom_pub.publish( odom_msg );
    }
}
