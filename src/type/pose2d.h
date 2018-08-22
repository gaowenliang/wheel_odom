#ifndef POSE2D_H
#define POSE2D_H

#define CON_PI_F 3.14159265359

#include <math.h>

template< class T >
class Pose2D
{
    public:
    Pose2D( )
    : x( 0 )
    , y( 0 )
    , yaw( 0 )
    {
    }
    Pose2D( T _x, T _y, T _yaw )
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
    inline Pose2D add( const T& dx, const T& dy, const T& dyaw ) const
    {
        return Pose2D( x + ( dx * cos( yaw ) - dy * sin( yaw ) ), //
                       y + ( dx * sin( yaw ) + dy * cos( yaw ) ), //
                       fmod( ( yaw + dyaw ), 2 * CON_PI_F ) );
    }

    T x;
    T y;
    T yaw; // yaw in rad
};

typedef Pose2D< double > Pose2Dd;
typedef Pose2D< float > Pose2Df;

#endif // POSE2D_H
