#ifndef __ENVIRE_PROJECTION_HPP__
#define __ENVIRE_PROJECTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/Grids.hpp>

#include <Eigen/Core>

namespace envire {
    class Projection : public Operator
    {
	ENVIRONMENT_ITEM( Projection )

    public:
	Projection();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	void addInput( Pointcloud* mesh ); 
	void addOutput( ElevationGrid* grid ); 

	bool updateAll();

	bool updateTraversibilityMap();
	bool updateElevationMap();
	bool interpolateMap(const std::string& type);
    };
}
#endif
