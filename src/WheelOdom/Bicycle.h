#ifndef BICYCLE_H
#define BICYCLE_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{
//           ^ x
//           |
//           |
//     y<----O
//
//      -       [0]
//      |       | |
//    length    | |
//      |       | |
//      -       [1]

class Bicycle : public WheelOdom
{
    public:
    Bicycle( ) {}
    Bicycle( double _length, double _width )
    : WheelOdom( _length, _width, 2 )
    , m_R( 0 )
    {
        m_lf = m_length / 2;
        m_lr = m_length / 2;
    }

    void calcOdom( );
    std::vector< double > velToWheelVel( const Pose2Dd pose )
    {
        return std::vector< double >( );
    }
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_lf, m_lr;
    double m_R;    // turn radius in m
    double m_beta; //
};

typedef boost::shared_ptr< Bicycle > BicyclePtr;
}
#endif // BICYCLE_H
