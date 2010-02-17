#ifndef __SCANMESHING_HPP__
#define __SCANMESHING_HPP__

#include "Core.hpp" 
#include "LaserScan.hpp" 
#include "TriMesh.hpp" 

namespace envire {
    class ScanMeshing : public Operator
    {
	static const std::string className;

	float maxEdgeLength;
	float remissionScaleFactor;

    public:
	ScanMeshing();

	ScanMeshing(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( LaserScan* scan ); 
	void addOutput( TriMesh* mesh ); 

	void setMaxEdgeLength( float value );
	bool updateAll();
    };
}

#endif
