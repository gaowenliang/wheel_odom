#include "OmniWheel.h"

#define m_speed0 speedIndex( 0 )
#define m_speed1 speedIndex( 1 )
#define m_speed2 speedIndex( 2 )

wheel_odom::OmniWheel::OmniWheel( ) {}

wheel_odom::OmniWheel::OmniWheel( double _length, double _width )
: WheelOdom( _length, _width, 3 )
, m_L( 0.0 )
{
    m_L = _length / 1.5;
}

void
wheel_odom::OmniWheel::calcOdom( )
{
    Eigen::Matrix3d F;
    F << 1 / sqrt( 3 ), -1 / sqrt( 3 ), 0, //
    1 / 3, 1 / 3, -2 / 3,                  //
    1 / ( 3 * m_L ), 1 / ( 3 * m_L ), 1 / ( 3 * m_L );

    Eigen::Vector3d vel;
    vel << m_speed0, m_speed1, m_speed2;

    Eigen::Vector3d vel_xyo = F * vel;

    m_vel = Pose2Dd( vel_xyo( 0 ), vel_xyo( 1 ), vel_xyo( 2 ) );
    m_pose = m_pose.add( vel_xyo( 0 ) * m_deltaT, vel_xyo( 1 ) * m_deltaT, vel_xyo( 2 ) * m_deltaT );
}

std::vector< double >
wheel_odom::OmniWheel::velToWheelVel( const Pose2Dd vel )
{
    // v = [R] * [V]
    Eigen::Vector3d vel_in;
    vel_in << vel.x, vel.y, vel.yaw;

    Eigen::Matrix3d F;
    F << 1 / sqrt( 3 ), -1 / sqrt( 3 ), 0, //
    1 / 3, 1 / 3, -2 / 3,                  //
    1 / ( 3 * m_L ), 1 / ( 3 * m_L ), 1 / ( 3 * m_L );

    Eigen::Matrix3d R;
    R = F.inverse( );

    Eigen::Vector3d vel_wheel = R * vel_in;

    std::vector< double > vels;
    vels.push_back( vel_wheel( 0 ) );
    vels.push_back( vel_wheel( 1 ) );
    vels.push_back( vel_wheel( 2 ) );
    return vels;
}
