#include "Bicycle.h"

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
#define m_speedF speedIndex( 0 )
#define m_speedB speedIndex( 0 )

void
wheel_odom::Bicycle::calcOdom( )
{
    m_beta = atan( m_lf / ( m_lf + m_lr ) * tan( m_steeringAngle ) );
}
