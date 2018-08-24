#include "MecanumWheel.h"

//           ^ x
//           |
//           |
//     y<----O
//
//      |   width  |
//    - [1]-------[0]
//    |      | |
//  length   |O|
//    |      | |
//    - [2]-------[3]
//
// config of Mecanum Wheel:
//      [//]-------[\\]
//           | |
//           | |
//           | |
//      [\\]-------[//]
//

#define m_speed0 speedIndex( 0 )
#define m_speed1 speedIndex( 1 )
#define m_speed2 speedIndex( 2 )
#define m_speed3 speedIndex( 3 )

void
wheel_odom::MecanumWheel::calcOdom( )
{
    /*
      Eigen::MatrixXd R( 4, 3, Eigen::RowMajor );
      R << 1, 1, -( m_a + m_b ), //
      1, -1, -( m_a + m_b ),     //
      1, 1, ( m_a + m_b ),       //
      1, -1, ( m_a + m_b );

      Eigen::MatrixXd F;
      F = ( R.transpose( ) * R ).inverse( ) * R.transpose( );
  */

    Eigen::MatrixXd F( 3, 4 );
    F << 1, 1, 1, 1, //
    1, -1, 1, -1,    //
    1 / ( m_a + m_b ), -1 / ( m_a + m_b ), -1 / ( m_a + m_b ), 1 / ( m_a + m_b );

    F = 0.25 * F;

    Eigen::Vector4d vel;
    vel << m_speed0, m_speed1, m_speed2, m_speed3;

    Eigen::Vector3d vel_xyo = F * vel;
    m_vel                   = Pose2Dd( vel_xyo( 0 ), vel_xyo( 1 ), vel_xyo( 2 ) );
    m_pose = m_pose.add( vel_xyo( 0 ) * m_deltaT, vel_xyo( 1 ) * m_deltaT, vel_xyo( 2 ) * m_deltaT );
}

std::vector< double >
wheel_odom::MecanumWheel::velToWheelVel( const Pose2Dd vel )
{
    // v = [R] * [V]

    double v0 = vel.x + vel.y + ( m_a + m_b ) * vel.yaw;
    double v1 = vel.x - vel.y - ( m_a + m_b ) * vel.yaw;
    double v2 = vel.x + vel.y - ( m_a + m_b ) * vel.yaw;
    double v3 = vel.x - vel.y + ( m_a + m_b ) * vel.yaw;

    std::vector< double > vels;
    vels.push_back( v0 );
    vels.push_back( v1 );
    vels.push_back( v2 );
    vels.push_back( v3 );
    return vels;
}
