#include "WheelOdomFactory.h"
#include "FrontWheel.h"
#include "RearWheel.h"
#include "Tricycle.h"

boost::shared_ptr< wheel_odom::WheelOdomFactory > wheel_odom::WheelOdomFactory::m_odomGen;

wheel_odom::WheelOdomFactory::WheelOdomFactory( ) {}

boost::shared_ptr< wheel_odom::WheelOdomFactory >
wheel_odom::WheelOdomFactory::newWheelOdom( )
{
    if ( m_odomGen.get( ) == 0 )
    {
        m_odomGen.reset( new WheelOdomFactory );
    }

    return m_odomGen;
}

wheel_odom::WheelOdomPtr
wheel_odom::WheelOdomFactory::init( wheel_odom::WheelModel type, double _length, double _width )
{
    switch ( type )
    {
        case FRONT_WHEEL:
        {
            FrontWheelPtr initial( new FrontWheel( _length, _width ) );
            return initial;
        }
        case REAR_WHEEL:
        {
            RearWheelPtr initial( new RearWheel( _length, _width ) );
            return initial;
        }
        case TRICYCLE:
        {
            TricyclePtr initial( new Tricycle( _length, _width ) );
            return initial;
        }
        default:
        {
            RearWheelPtr initial( new RearWheel( _length, _width ) );
            return initial;
        }
    }
    return WheelOdomPtr( );
}
