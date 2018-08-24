#include "FrontWheel.h"

//           ^ x
//           |
//           |
//     y<----O
//
//      |   width  |
//    - [1]-------[0]
//    |      | |
//  length   | |
//    |      | |
//    -
//
#define m_speedFR speedIndex( 0 )
#define m_speedFL speedIndex( 1 )

wheel_odom::FrontWheel::FrontWheel( )
: m_R( 0 )
{
}

wheel_odom::FrontWheel::FrontWheel( double _length, double _width )
: WheelOdom( _length, _width, 2 )
, m_R( 0.0 )
{
}

void
wheel_odom::FrontWheel::calcOdom( )
{
    double avgSpeedF; //  Average speed of the front two wheels , m/s

    avgSpeedF = ( m_speedFR + m_speedFL ) / 2;

    double dx = 0, dy = 0, dtheta = 0; // in local frame
    if ( std::abs( m_steeringAngle ) < 1e-6 )
    {
        dx     = avgSpeedF * m_deltaT;
        dy     = 0;
        dtheta = 0;
    }
    else
    {
        m_R = m_length / sin( std::abs( m_steeringAngle ) );

        double turning_phi = avgSpeedF * m_deltaT / m_R;
        dx                 = m_R * sin( turning_phi );
        if ( m_steeringAngle > 0 )
        {
            dy = m_R - m_R * cos( turning_phi );
        }
        else if ( m_steeringAngle < 0 )
        {
            dy = -( m_R - m_R * cos( turning_phi ) );
        }
        dtheta = avgSpeedF / m_length * sin( m_steeringAngle ) * m_deltaT;
    }

    m_pose = m_pose.add( dx, dy, dtheta );
    m_vel  = Pose2Dd( dx / m_deltaT, dy / m_deltaT, dtheta / m_deltaT );
}
