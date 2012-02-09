#ifndef __ENVIRE_SURFACERECONSTRUCTION_HPP__
#define __ENVIRE_SURFACERECONSTRUCTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/TriMesh.hpp>

namespace envire {
    class SurfaceReconstruction : public Operator
    {
    public:
	static const std::string className;

	SurfaceReconstruction();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( Pointcloud* input ); 
	void addOutput( TriMesh* output ); 

	bool updateAll();
    };
}
#endif
