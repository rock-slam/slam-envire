#ifndef __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__
#define __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>

namespace envire {
    class SimplifyPointcloud : public Operator
    {
	ENVIRONMENT_ITEM( SimplifyPointcloud )

    public:
	SimplifyPointcloud();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	void addInput( Pointcloud* input ); 
	void addOutput( Pointcloud* output ); 

	bool updateAll();

	void setComputeSpacing( bool value ) { computeSpacing = value; }
	void setSimplifyCellSize( double value ) { simplifyCellSize = value; }
	void setOutlierPercentage( double value ) { outlierPercentage = value; }
	void setSmoothNeighbours( int value ) { smoothNeighbours = value; }

    private:
	void initDefaults();

	bool computeSpacing;
	double simplifyCellSize;
	double outlierPercentage;
	size_t smoothNeighbours;
    };
}
#endif
