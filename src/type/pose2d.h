#ifndef POSE2D_H
#define POSE2D_H

#define CON_PI_F 3.14159265359

#include <math.h>

class Pose2D
{
    public:
    Pose2D( )
    : x( 0 )
    , y( 0 )
    , yaw( 0 )
    {
    }
    Pose2D( double _x, double _y, double _yaw )
    : x( _x )
    , y( _y )
    , yaw( _yaw )
    {
    }

    inline Pose2D& operator=( const Pose2D& other )
    {
        if ( this != &other )
        {
            x   = other.x;
            y   = other.y;
            yaw = other.yaw;
        }
        return *this;
    }
    inline Pose2D operator+( const Pose2D& dpose ) const
    {
        return Pose2D( x + ( dpose.x * cos( yaw ) - dpose.y * sin( yaw ) ), //
                       y + ( dpose.x * sin( yaw ) + dpose.y * cos( yaw ) ), //
                       fmod( ( yaw + dpose.yaw ), 2 * CON_PI_F ) );
    }
    inline Pose2D add( const double& dx, const double& dy, const double& dyaw ) const
    {
        return Pose2D( x + ( dx * cos( yaw ) - dy * sin( yaw ) ), //
                       y + ( dx * sin( yaw ) + dy * cos( yaw ) ), //
                       fmod( ( yaw + dyaw ), 2 * CON_PI_F ) );
    }

    double x;
    double y;
    double yaw; // yaw in rad
};

#endif // POSE2D_H
