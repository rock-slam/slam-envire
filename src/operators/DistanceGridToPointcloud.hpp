#ifndef __ENVIRE_DISTANCEGRIDTOPOINTCLOUD_HPP__
#define __ENVIRE_DISTANCEGRIDTOPOINTCLOUD_HPP__

#include <envire/Core.hpp>

namespace envire
{
    class DistanceGridToPointcloud : public Operator
    {
	ENVIRONMENT_ITEM( DistanceGridToPointcloud )
	
    public:
	DistanceGridToPointcloud() {};
	DistanceGridToPointcloud( Serialization& so ) : Operator( so ) {}
	void serialize( Serialization& so ) { Operator::serialize( so ); }

	bool updateAll();
    };
}

#endif


