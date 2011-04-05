#ifndef __ENVIRE__TRAVERSIBILITY_HPP__
#define __ENVIRE__TRAVERSIBILITY_HPP__

#include <envire/Core.hpp>

namespace envire
{
    class Traversability : public Operator
    {
	ENVIRONMENT_ITEM( Traversability )
    public:
	Traversability( Serialization &so ) : Operator( so ) {}
	void serialize( Serialization &so ) { Operator::serialize( so ) ;}

	bool updateAll();
    };
}

#endif
