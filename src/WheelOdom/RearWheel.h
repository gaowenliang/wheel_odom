#ifndef RearWheel_H
#define RearWheel_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{
//
//      -  |   width  |
//      |       | |
//    length    | |
//      |       | |
//      -  [1]-------[0]
// 1 Speed of the back right wheel , m/s
// 0 Speed of the back left wheel , m/s

class RearWheel : public WheelOdom
{
    public:
    RearWheel( );
    RearWheel( double _length, double _width );

    void calcOdom( );
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
};

typedef boost::shared_ptr< RearWheel > RearWheelPtr;
}

#endif // RearWheel_H
