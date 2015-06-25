#ifndef __ENVIRE_MLSPROJECTION_HPP__
#define __ENVIRE_MLSPROJECTION_HPP__

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/MLSGrid.hpp>

#include <Eigen/Core>
#include <envire/core/Operator.hpp>
#include <Eigen/src/Geometry/AlignedBox.h>

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
	void setDefaultUncertainty( double uncertainty ) { defaultUncertainty = uncertainty; }
    
        /** 
         * Only samples within the area of interest will be projected. 
         * All parameters are in the transformation frame of the mls grid.
         */
        void setAreaOfInterest(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);
        void unsetAreaOfInterest();

    protected:
	void projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );
	void projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc, envire::MultiLevelSurfaceGrid* main_grid = NULL );

	bool withUncertainty;
	double defaultUncertainty;
        bool use_boundary_box;
        Eigen::AlignedBox<double,3> boundary_box;

    private:
	TransformWithUncertainty C_m2g;
    };
}
#endif
