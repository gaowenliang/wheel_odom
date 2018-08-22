#ifndef FrontWheel_H
#define FrontWheel_H

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

class FrontWheel : public WheelOdom
{
    public:
    FrontWheel( );
    FrontWheel( double _length, double _width );

    void calcOdom( );
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
};

typedef boost::shared_ptr< FrontWheel > FrontWheelPtr;
}

#endif // FrontWheel_H
