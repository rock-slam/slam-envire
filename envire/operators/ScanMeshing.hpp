#ifndef __SCANMESHING_HPP__
#define __SCANMESHING_HPP__

#include <envire/Core.hpp>
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>

namespace envire {
    class ScanMeshing : public Operator
    {
	double maxEdgeLength;
	double remissionScaleFactor;
	long remissionMarkerThreshold;
	double minRange;

	bool extractMarkers;

    public:
	static const std::string className;

	ScanMeshing();

	ScanMeshing(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( LaserScan* scan ); 
	void addOutput( TriMesh* mesh ); 

	void setMaxEdgeLength( double value );
	void setRemissionScaleFactor( double value );
	void setRemissionMarkerThreshold( long value );
	void setMinRange( double value );
	void setExtractMarkers( bool value );
	bool updateAll();

	void setDefaultConfiguration();
    };
}

#endif
