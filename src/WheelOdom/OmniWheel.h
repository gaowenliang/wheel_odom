#ifndef OMNIWHEEL_H
#define OMNIWHEEL_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{

//      |   width  |
//    - [1]-------[0]
//    |      | |
//  length   | |
//    |      | |
//    -
//
// 0 Speed of the front right wheel , m/s
// 1  Speed of the front left wheel , m/s

class OmniWheel : public WheelOdom
{
    public:
    OmniWheel( ) {}
    OmniWheel( double _length, double _width )
    : WheelOdom( _length, _width, 2 )
    , m_R( 0.0 )
    {
    }

    void calcOdom( );
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
};

typedef boost::shared_ptr< OmniWheel > OmniWheelPtr;
}
#endif // OMNIWHEEL_H
