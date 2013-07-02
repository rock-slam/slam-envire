#ifndef __SCANMESHING_HPP__
#define __SCANMESHING_HPP__

#include <envire/Core.hpp>
#include <envire/core/Operator.hpp>
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>

namespace envire 
{
    class ScanMeshing : public Operator
    {
	ENVIRONMENT_ITEM( ScanMeshing )

	double maxEdgeAngle;
	double maxEdgeLength;
	double remissionScaleFactor;
	long remissionMarkerThreshold;
	double minRange;
	double maxRange;

	bool extractMarkers;

    public:
	ScanMeshing();
        
	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	void addInput( LaserScan* scan ); 
	void addOutput( TriMesh* mesh ); 

	void setMaxEdgeLength( double value );
	void setRemissionScaleFactor( double value );
	void setRemissionMarkerThreshold( long value );
	void setMinRange( double value );
	void setMaxRange( double value );
	void setExtractMarkers( bool value );
	bool updateAll();

	void setDefaultConfiguration();
    };
}

#endif
