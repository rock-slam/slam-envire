#ifndef __ENVIRE_MLSPROJECTION_HPP__
#define __ENVIRE_MLSPROJECTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/MLSGrid.hpp>

#include <Eigen/Core>

namespace envire {
    class MLSProjection : public Operator
    {
	ENVIRONMENT_ITEM( MLSProjection )

    public:
	MLSProjection();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	void addInput( Pointcloud* mesh ); 
	void addOutput( MultiLevelSurfaceGrid* grid ); 

	bool updateAll();

	void useUncertainty( bool use ) { withUncertainty = use; }
	void useNegativeInformation( bool use ) { m_negativeInformation = use; }
	void setDefaultUncertainty( double uncertainty ) { defaultUncertainty = uncertainty; }

    protected:
	void projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );
	void projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );

	bool withUncertainty;
	bool m_negativeInformation;
	double defaultUncertainty;

    private:
	TransformWithUncertainty C_m2g;
    };
}
#endif
