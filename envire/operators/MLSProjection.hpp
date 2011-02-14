#ifndef __ENVIRE_MLSPROJECTION_HPP__
#define __ENVIRE_MLSPROJECTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

#include <Eigen/Core>

namespace envire {
    class MLSProjection : public Operator
    {
	ENVIRONMENT_ITEM( MLSProjection )

    public:
	MLSProjection();

	MLSProjection(Serialization& so);
	void serialize(Serialization& so);

	void addInput( Pointcloud* mesh ); 
	void addOutput( MultiLevelSurfaceGrid* grid ); 

	bool updateAll();

	void useUncertainty( bool use ) { withUncertainty = use; }

    protected:
	void projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );
	void projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );

	bool withUncertainty;
    };
}
#endif
