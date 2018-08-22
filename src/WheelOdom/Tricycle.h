#ifndef TRICYCLE_H
#define TRICYCLE_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{

//          |   width  |
//      -       [0]
//      |       | |
//    length    | |
//      |       | |
//      -  [1]-------[2]
// 1 Speed of the back right wheel , m/s
// 0 Speed of the back left wheel , m/s

class Tricycle : public WheelOdom
{
    public:
    Tricycle( ) {}
    Tricycle( double _length, double _width )
    : WheelOdom( _length, _width, 3 )
    , m_R( 0 )
    {
    }

    void calcOdom( ) {}
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
};
typedef boost::shared_ptr< Tricycle > TricyclePtr;
}
#endif // TRICYCLE_H
