#ifndef WHEELODOM_H
#define WHEELODOM_H

#include "../type/pose2d.h"
#include "eigen3/Eigen/Eigen"
#include <boost/shared_ptr.hpp>

namespace wheel_odom
{

enum WheelModel
{
    FRONT_WHEEL = 0,
    REAR_WHEEL,
    TRICYCLE,
    BICYCLE,
    OMNI_WHEEL,
    MECANUM_WHEEL
};

class WheelOdom
{
    public:
    WheelOdom( );
    WheelOdom( double _length, double _width, int num );

    double steeringAngle( ) const;
    double speedIndex( int index ) const;

    void setsteeringAngle( double steeringAngle );
    void setSpeedIndex( const double speed, const int index );
    void setOmegaIndex( const double _omega, const int index );
    void setRIndex( const double _R, const int index );
    void setDt( double deltaT );

    // Inverse Kinematic Problem
    virtual void calcOdom( ) = 0;
    // Forward Kinematic Problem
    virtual std::vector< double > velToWheelVel( const Pose2Dd pose ) = 0;

    virtual Pose2Dd getPose( Eigen::Matrix3d exR, Eigen::Vector3d exT ) = 0;
    virtual Pose2Dd getVel( Eigen::Matrix3d exR, Eigen::Vector3d exT )  = 0;

    Pose2Dd getPose( ) const;
    Pose2Dd getVel( ) const;

    void setPose( const Pose2Dd& value );

    void setLength( double length );
    void setWidth( double width );

    protected:
    WheelModel model;

    protected:
    double m_length; // m_vehicleWheelbase;
    double m_width;  // m_vehicleWheelDis;

    protected:
    double m_deltaT;        //  dt, sec
    double m_steeringAngle; //  Steering Anglel , rad
    int m_numOfWheel;
    std::vector< double > m_speeds;
    std::vector< double > m_wheelRs;
    std::vector< double > m_wheelOmegas;

    Pose2Dd m_pose;
    Pose2Dd m_vel;
};

typedef boost::shared_ptr< WheelOdom > WheelOdomPtr;
typedef boost::shared_ptr< const WheelOdom > WheelOdomConstPtr;
}
#endif // WHEELODOM_H
