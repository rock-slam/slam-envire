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

    protected:
	void updateCell(MultiLevelSurfaceGrid* grid, size_t m, size_t n, double mean, double stdev );

	double gapSize;
	double thickness;
    };
}
#endif
