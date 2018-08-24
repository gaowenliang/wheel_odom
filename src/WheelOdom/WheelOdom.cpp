#include "WheelOdom.h"

wheel_odom::WheelOdom::WheelOdom( ) {}

wheel_odom::WheelOdom::WheelOdom( double _length, double _width, int num )
: m_length( _length )
, m_width( _width )
, m_numOfWheel( num )
{
    m_speeds.resize( m_numOfWheel );
    m_wheelRs.resize( m_numOfWheel );
    m_wheelOmegas.resize( m_numOfWheel );
}

double
wheel_odom::WheelOdom::steeringAngle( ) const
{
    return m_steeringAngle;
}

double
wheel_odom::WheelOdom::speedIndex( int index ) const
{
    return m_speeds[index];
}

void
wheel_odom::WheelOdom::setsteeringAngle( double steeringAngle )
{
    m_steeringAngle = steeringAngle;
}

void
wheel_odom::WheelOdom::setSpeedIndex( const double speed, const int index )
{
    m_speeds[index] = speed;
}

void
wheel_odom::WheelOdom::setOmegaIndex( const double _omega, const int index )
{
    m_wheelOmegas[index] = _omega;
    m_speeds[index]      = m_wheelOmegas[index] * m_wheelRs[index];
}

void
wheel_odom::WheelOdom::setRIndex( const double _R, const int index )
{
    m_wheelRs[index] = _R;
}

void
wheel_odom::WheelOdom::setDt( double deltaT )
{
    m_deltaT = deltaT;
}

Pose2Dd
wheel_odom::WheelOdom::getPose( ) const
{
    return m_pose;
}

Pose2Dd
wheel_odom::WheelOdom::getVel( ) const
{
    return m_vel;
}

void
wheel_odom::WheelOdom::setPose( const Pose2Dd& value )
{
    m_pose = value;
}

void
wheel_odom::WheelOdom::setLength( double length )
{
    m_length = length;
}

void
wheel_odom::WheelOdom::setWidth( double width )
{
    m_width = width;
}
