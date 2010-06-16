#ifndef __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__
#define __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__

#include "Core.hpp" 
#include "Pointcloud.hpp" 
#include "Grids.hpp" 

#include <Eigen/Core>

namespace envire {
    class SimplifyPointcloud : public Operator
    {
    public:
	static const std::string className;

	SimplifyPointcloud();

	SimplifyPointcloud(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( Pointcloud* input ); 
	void addOutput( Pointcloud* output ); 

	bool updateAll();
    };
}
#endif
