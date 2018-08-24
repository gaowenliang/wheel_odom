#ifndef TRICYCLE_H
#define TRICYCLE_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{
//           ^ x
//           |
//           |
//     y<----O
//
//          |   width  |
//      -       [0]
//      |       | |
//    length    | |
//      |       | |
//      -  [1]-------[2]

class Tricycle : public WheelOdom
{
    public:
    Tricycle( );
    Tricycle( double _length, double _width );

    void calcOdom( );
    std::vector< double > velToWheelVel( const Pose2Dd pose )
    {
        return std::vector< double >( );
    }
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
    double m_omega;
};

typedef boost::shared_ptr< Tricycle > TricyclePtr;
}
#endif // TRICYCLE_H
