#include "WheelOdom2.h"

wheel_odom::WheelOdom2::WheelOdom2( )
: m_R( 0 )
{
}

wheel_odom::WheelOdom2::WheelOdom2( double _length, double _width )
: m_length( _length )
, m_width( _width )
, m_R( 0 )
{
}

double
wheel_odom::WheelOdom2::steeringAngle( ) const
{
    return m_steeringAngle;
}

double
wheel_odom::WheelOdom2::speedFL( ) const
{
    return m_speedFL;
}

double
wheel_odom::WheelOdom2::speedFR( ) const
{
    return m_speedFR;
}

double
wheel_odom::WheelOdom2::speedBL( ) const
{
    return m_speedBL;
}

double
wheel_odom::WheelOdom2::speedBR( ) const
{
    return m_speedBR;
}

void
wheel_odom::WheelOdom2::setsteeringAngle( double steeringAngle )
{
    m_steeringAngle = steeringAngle;
}

void
wheel_odom::WheelOdom2::setSpeedFL( double speedFL )
{
    m_speedFL = speedFL;
}

void
wheel_odom::WheelOdom2::setSpeedFR( double speedFR )
{
    m_speedFR = speedFR;
}

void
wheel_odom::WheelOdom2::setSpeedBL( double speedBL )
{
    m_speedBL = speedBL;
}

void
wheel_odom::WheelOdom2::setSpeedBR( double speedBR )
{
    m_speedBR = speedBR;
}

void
wheel_odom::WheelOdom2::setDt( double deltaT )
{
    m_deltaT = deltaT;
}

void
wheel_odom::WheelOdom2::calcOdom( )
{
    m_avgSpeedF = ( m_speedFL + m_speedFR ) / 2;

    double dx = 0, dy = 0, dtheta = 0;
    if ( std::abs( m_steeringAngle ) < 1e-6 )
    {
        dx     = m_avgSpeedF * m_deltaT;
        dy     = 0;
        dtheta = 0;
    }
    else
    {
        m_R = m_length / sin( std::abs( m_steeringAngle ) );

        double turning_phi = m_avgSpeedF * m_deltaT / m_R;
        dx                 = m_R * sin( turning_phi );
        if ( m_steeringAngle > 0 )
        {
            dy = m_R - m_R * cos( turning_phi );
        }
        else if ( m_steeringAngle < 0 )
        {
            dy = -( m_R - m_R * cos( turning_phi ) );
        }
        dtheta = m_avgSpeedF / m_length * sin( m_steeringAngle ) * m_deltaT;
    }

    pose = pose.add( dx, dy, dtheta );
    vel  = Pose2D( dx / m_deltaT, dy / m_deltaT, dtheta / m_deltaT );
}

Pose2D
wheel_odom::WheelOdom2::getPose( ) const
{
    return pose;
}

Pose2D
wheel_odom::WheelOdom2::getVel( ) const
{
    return vel;
}

void
wheel_odom::WheelOdom2::setPose( const Pose2D& value )
{
    pose = value;
}

void
wheel_odom::WheelOdom2::setLength( double length )
{
    m_length = length;
}

void
wheel_odom::WheelOdom2::setWidth( double width )
{
    m_width = width;
}
