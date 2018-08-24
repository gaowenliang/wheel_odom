#include <math.h>
#include <ros/ros.h>
#include <wheel_odom/steeringAngle.h>
#include <wheel_odom/wheelSpeeds.h>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "WheelPub" );
    ros::NodeHandle n;

    ros::Publisher speed_pub   = n.advertise< wheel_odom::wheelSpeeds >( "wheelSpeeds", 1 );
    ros::Publisher chassis_pub = n.advertise< wheel_odom::steeringAngle >( "steering", 1 );

    ros::Rate loop( 10 );
    while ( ros::ok( ) )
    {
        loop.sleep( );

        ros::Time now_t = ros::Time::now( );

        wheel_odom::steeringAngle chassisState_;
        chassisState_.header.stamp    = now_t;
        chassisState_.header.frame_id = "wheel";
        chassisState_.steering        = std::abs( sin( now_t.toSec( ) ) );
        chassis_pub.publish( chassisState_ );

        double randn1 = 0.1 * rand( ) / double( RAND_MAX );
        double randn2 = 0.1 * rand( ) / double( RAND_MAX );
        double randn3 = 0.1 * rand( ) / double( RAND_MAX );
        double randn4 = 0.1 * rand( ) / double( RAND_MAX );

        wheel_odom::wheelSpeeds wheelSpeeds_;
        wheelSpeeds_.header.stamp    = now_t;
        wheelSpeeds_.header.frame_id = "wheel";
        if ( 1 )
        {
            wheelSpeeds_.speedLB = 1.2 + randn1;
            wheelSpeeds_.speedRB = 1.3 + randn2;
            wheelSpeeds_.speedLF = 1.2 + randn3;
            wheelSpeeds_.speedRF = 1.4 + randn4;
        }
        if ( 0 )
        {
            wheelSpeeds_.speedLF = 10 * randn3;
            wheelSpeeds_.speedLB = 10 * randn1;
            wheelSpeeds_.speedRB = 10 * randn2;
            wheelSpeeds_.speedRF = 10 * randn4;
        }
        speed_pub.publish( wheelSpeeds_ );

        std::cout << "speed " << wheelSpeeds_.speedLB << " " << wheelSpeeds_.speedRB << " "
                  << wheelSpeeds_.speedLF << " " << wheelSpeeds_.speedRF << "\n";
    }

    return 0;
}
