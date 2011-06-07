#ifndef __ENVIRE_DISTANCEGRIDTOPOINTCLOUD_HPP__
#define __ENVIRE_DISTANCEGRIDTOPOINTCLOUD_HPP__

#include <envire/Core.hpp>

namespace envire
{
    class DistanceGridToPointcloud : public Operator
    {
	ENVIRONMENT_ITEM( DistanceGridToPointcloud )
	
    public:
	DistanceGridToPointcloud() : uncertaintyFactor(0.1) {};
	DistanceGridToPointcloud( Serialization& so ) : Operator( so ) {}
	void serialize( Serialization& so ) { Operator::serialize( so ); }

	bool updateAll();

	double setUncertaintyFactor( double f ) { uncertaintyFactor = f; }

    private:
	double uncertaintyFactor;
    };
}

#endif


