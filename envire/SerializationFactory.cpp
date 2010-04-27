#include "Core.hpp"

#include "LaserScan.hpp"
#include "TriMesh.hpp"
#include "ScanMeshing.hpp"
#include "Projection.hpp"
#include "Pointcloud.hpp"
#include "Grids.hpp"

using namespace envire;

template<class T> EnvironmentItem* create(Serialization &so) 
{
    T* o = new T(so);
    return o;
}

EnvironmentItem* SerializationFactory::createObject(const std::string& className, Serialization& so)
{
    static bool initialized = false;
    if( !initialized )
    {
	addClass(FrameNode::className, &create<FrameNode> );
	addClass(Pointcloud::className, &create<Pointcloud> );
	addClass(LaserScan::className, &create<LaserScan> );
	addClass(TriMesh::className, &create<TriMesh> );
	
	//addClass(Grid<unsigned char>::className, &create<Grid<unsigned char> >);
	//addClass(Grid<uint16_t>::className, &create<Grid<uint16_t> >);
	//addClass(Grid<int16_t>::className, &create<Grid<int16_t> >);
	//addClass(Grid<uint32_t>::className, &create<Grid<uint32_t> >);
	//addClass(Grid<int32_t>::className, &create<Grid<int32_t> >);
	//addClass(Grid<float>::className, &create<Grid<float> >);
	//addClass(Grid<double>::className, &create<Grid<double> >);
	
	addClass(ConfidenceGrid::className, &create<ConfidenceGrid>);
	addClass(TraversabilityGrid::className, &create<TraversabilityGrid>);
	addClass(ElevationGrid::className, &create<ElevationGrid>);
	addClass(OccupancyGrid::className, &create<OccupancyGrid>);
	addClass(ImageRGB24::className, &create<ImageRGB24>);
	
	addClass(ScanMeshing::className, &create<ScanMeshing> );
	addClass(Projection::className, &create<Projection> );

	initialized = true;
    }

    if( classMap.count( className ) == 0 )
    {
	std::cerr << "could not find class of type " << className << std::endl;
	std::cerr << "has the class been added to the envire/SerializationFactory.cpp file?" << std::endl;
	throw std::runtime_error("could not find class of type " + className );
    }

    Factory f = classMap[className];
    EnvironmentItem* envItem = (*f)(so);

    return envItem;
}

void SerializationFactory::addClass( const std::string& className, Factory f )
{
    classMap[className] = f;
}

