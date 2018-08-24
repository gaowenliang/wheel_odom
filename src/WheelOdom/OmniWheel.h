#ifndef OMNIWHEEL_H
#define OMNIWHEEL_H

#include "../WheelOdom/WheelOdom.h"
#include <cmath>

namespace wheel_odom
{

//           ^ x
//           |
//           |
//     y<----O
//
//   0.5*sqrt(3)*L
//      |< ------>|
//               <\
//     [1]-------[0]
//     /    | |
//   \/     | |
//          | |
//          [2]->

class OmniWheel : public WheelOdom
{
    public:
    OmniWheel( );
    OmniWheel( double _length, double _width );

    void calcOdom( );
    std::vector< double > velToWheelVel( const Pose2Dd vel );
    Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }
    Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT ) { return Pose2Dd( ); }

    private:
    double m_L; // turn radius in m
};

typedef boost::shared_ptr< OmniWheel > OmniWheelPtr;
}
#endif // OMNIWHEEL_H
