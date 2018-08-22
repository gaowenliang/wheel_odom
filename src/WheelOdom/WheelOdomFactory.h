#ifndef WHEELODOMFACTORY_H
#define WHEELODOMFACTORY_H

#include "WheelOdom.h"
#include <boost/shared_ptr.hpp>

namespace wheel_odom
{

class WheelOdomFactory
{
    public:
    WheelOdomFactory( );

    static boost::shared_ptr< WheelOdomFactory > newWheelOdom( );
    WheelOdomPtr init( WheelModel type, double _length, double _width );

    private:
    static boost::shared_ptr< WheelOdomFactory > m_odomGen;
};
}

#endif // WHEELODOMFACTORY_H
