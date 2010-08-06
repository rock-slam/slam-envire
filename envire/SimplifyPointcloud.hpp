#ifndef __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__
#define __ENVIRE_SIMPLIFYPOINTCLOUD_HPP__

#include "Core.hpp" 
#include "Pointcloud.hpp" 

namespace envire {
    class SimplifyPointcloud : public Operator
    {
    public:
	static const std::string className;

	SimplifyPointcloud();

	SimplifyPointcloud(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

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
