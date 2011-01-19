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

	void setGapSize( double gapSize ) { this->gapSize = gapSize; }
	double getGapSize() const { return gapSize; }

	void setHorizontalPatchThickness( double thickness ) { this->thickness = thickness; }
	double getHorizontalPatchThickness() const { return thickness; }

	MLSProjection* clone() const;
	void set( EnvironmentItem* other );

	void useUncertainty( bool use ) { withUncertainty = use; }

    protected:
	void updateCell(MultiLevelSurfaceGrid* grid, size_t m, size_t n, double mean, double stdev );
	void projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );
	void projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc );

	double gapSize;
	double thickness;
	bool withUncertainty;
    };
}
#endif
