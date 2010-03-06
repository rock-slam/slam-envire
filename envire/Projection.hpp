#ifndef __ENVIRE_PROJECTION_HPP__
#define __ENVIRE_PROJECTION_HPP__

#include "Core.hpp" 
#include "TriMesh.hpp" 
#include "Grid.hpp" 

#include <Eigen/Core>

namespace envire {
    class Projection : public Operator
    {

    public:
	static const std::string className;

	Projection();

	Projection(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( TriMesh* mesh ); 
	void addOutput( Grid* grid ); 

	bool updateAll();
    };
}

#endif
