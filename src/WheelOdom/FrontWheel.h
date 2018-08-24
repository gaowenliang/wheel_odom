#ifndef FrontWheel_H
#define FrontWheel_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{
//           ^ x
//           |
//           |
//     y<----O
//
//      |   width  |
//    - [1]---O---[0]
//    |      | |
//  length   | |
//    |      | |
//    -
//
class FrontWheel : public WheelOdom
{
    public:
    FrontWheel( );
    FrontWheel( double _length, double _width );

    void calcOdom( );
    std::vector< double > velToWheelVel( const Pose2Dd vel )
    {
        return std::vector< double >( );
    }
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_R; // turn radius in m
};

typedef boost::shared_ptr< FrontWheel > FrontWheelPtr;
}

#endif // FrontWheel_H
