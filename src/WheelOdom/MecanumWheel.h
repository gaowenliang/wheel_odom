#ifndef MECANUMWHEEL_H
#define MECANUMWHEEL_H

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
//    - [1]-------[0]
//    |      | |
//  length   | |
//    |      | |
//    - [2]-------[3]
//

class MecanumWheel : public WheelOdom
{
    public:
    MecanumWheel( ) {}
    MecanumWheel( double _length, double _width )
    : WheelOdom( _length, _width, 4 )
    {
        m_a = m_width / 2;
        m_b = m_length / 2;
    }

    void calcOdom( );
    std::vector< double > velToWheelVel( const Pose2Dd pose );
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_omega;

    double m_a;
    double m_b;
};

typedef boost::shared_ptr< MecanumWheel > MecanumWheelPtr;
}

#endif // MECANUMWHEEL_H
