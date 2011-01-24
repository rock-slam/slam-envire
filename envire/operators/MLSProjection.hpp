#ifndef __ENVIRE_MLSPROJECTION_HPP__
#define __ENVIRE_MLSPROJECTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

#include <Eigen/Core>

namespace envire {
    class MLSProjection : public Operator
    {

    public:
	static const std::string className;

	MLSProjection();

	MLSProjection(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( Pointcloud* mesh ); 
	void addOutput( MultiLevelSurfaceGrid* grid ); 

	bool updateAll();

	MLSProjection* clone() const;
	void set( EnvironmentItem* other );

	void useUncertainty( bool use ) { withUncertainty = use; }

    protected:
	void projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );
	void projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );

	bool withUncertainty;
    };
}
#endif
