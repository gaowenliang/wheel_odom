#include "Tricycle.h"

//           ^ x
//           |
//           |
//     y<----O
//
//          |   width  |
//      -       [0]
//      |       | |
//    length    | |
//      |       | |
//      -  [1]---O---[2]

#define m_speedF speedIndex( 0 )

void
wheel_odom::Tricycle::calcOdom( )
{

    double dx = 0, dy = 0, dtheta = 0;
    if ( std::abs( m_steeringAngle ) < 1e-6 )
    {
        dx     = m_speedF * m_deltaT;
        dy     = 0;
        dtheta = 0;
    }
    else
    {
        m_R = m_length * tan( 0.5 * CON_PI_F - m_steeringAngle );

        double vel_x_b = m_speedF * cos( m_steeringAngle );
        double vel_y_b = 0;

        dx = vel_x_b * m_deltaT;
        dy = vel_y_b * m_deltaT;

        m_omega = m_speedF / m_length * sin( m_steeringAngle );
        dtheta  = m_omega * m_deltaT;
    }

    m_pose = m_pose.add( dx, dy, dtheta );
    m_vel  = Pose2Dd( dx / m_deltaT, dy / m_deltaT, dtheta / m_deltaT );
}

wheel_odom::Tricycle::Tricycle( ) {}

wheel_odom::Tricycle::Tricycle( double _length, double _width )
: WheelOdom( _length, _width, 1 )
, m_R( 0 )
{
}
