#ifndef WHEELODOM2_H
#define WHEELODOM2_H

#include "../type/pose2d.h"
#include <cmath>

namespace wheel_odom
{

class WheelOdom2
{
    public:
    WheelOdom2( );
    WheelOdom2( double _length, double _width );

    double steeringAngle( ) const;
    double speedFL( ) const;
    double speedFR( ) const;
    double speedBL( ) const;
    double speedBR( ) const;

    void setsteeringAngle( double steeringAngle );
    void setSpeedFL( double speedFL );
    void setSpeedFR( double speedFR );
    void setSpeedBL( double speedBL );
    void setSpeedBR( double speedBR );
    void setDt( double deltaT );

    void calcOdom( );

    Pose2D getPose( ) const;
    Pose2D getVel( ) const;

    void setPose( const Pose2D& value );

    void setLength( double length );
    void setWidth( double width );

    private:
    double m_length; // m_vehicleWheelbase;
    double m_width;  // m_vehicleWheelDis;

    private:
    double m_deltaT;
    double m_steeringAngle;
    double m_speedFL;   //  Speed of the front left wheel , m/s
    double m_speedFR;   //  Speed of the front right wheel , m/s
    double m_speedBL;   //  Speed of the back left wheel , m/s
    double m_speedBR;   //  Speed of the back right wheel , m/s
    double m_avgSpeedF; //  Average speed of the front two wheels , m/s

    Pose2D pose;
    Pose2D vel;

    private:
    double m_R; // turn radius in m
};
}

#endif // WHEELODOM2_H
