#include "RearWheel.h"

//               ^ x
//               |
//               |
//         y<----O
//
//      -  |   width  |
//      |       | |
//    length    | |
//      |       | |
//      -  [1]-------[0]
#define m_speedBR speedIndex( 0 )
#define m_speedBL speedIndex( 1 )

wheel_odom::RearWheel::RearWheel( )
: m_R( 0 )
{
}

wheel_odom::RearWheel::RearWheel( double _length, double _width )
: WheelOdom( _length, _width, 2 )
, m_R( 0 )
{
}

void
wheel_odom::RearWheel::calcOdom( )
{
    double avgSpeedB; //  Average speed of the front two wheels , m/s
    avgSpeedB = ( m_speedBL + m_speedBR ) / 2;

    double dx, dy, dtheta;
    double omega;
    if ( std::abs( avgSpeedB - m_speedBL ) < 0.01 )
    {
        omega  = 0;
        dx     = avgSpeedB * m_deltaT;
        dy     = avgSpeedB * m_deltaT;
        dtheta = 0;
    }
    else
    {
        omega = ( m_speedBR - m_speedBL ) / m_width;
        m_R   = 0.5 * m_width * ( m_speedBR + m_speedBL ) / ( m_speedBR - m_speedBL );

        dx     = avgSpeedB * m_deltaT;
        dy     = 0;
        dtheta = omega * m_deltaT;
    }

    m_pose = m_pose.add( dx, dy, dtheta );
    m_vel  = Pose2Dd( dx / m_deltaT, dy / m_deltaT, dtheta / m_deltaT );
}
