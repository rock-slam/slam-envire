#ifndef __ENVIRE_SURFACERECONSTRUCTION_HPP__
#define __ENVIRE_SURFACERECONSTRUCTION_HPP__

#include "Core.hpp" 
#include "Pointcloud.hpp" 
#include "TriMesh.hpp" 

namespace envire {
    class SurfaceReconstruction : public Operator
    {
    public:
	static const std::string className;

	SurfaceReconstruction();

	SurfaceReconstruction(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( Pointcloud* input ); 
	void addOutput( TriMesh* output ); 

	bool updateAll();
    };
}
#endif
