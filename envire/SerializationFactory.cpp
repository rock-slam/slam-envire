#include "Core.hpp"

#include "LaserScan.hpp"
#include "TriMesh.hpp"
#include "ScanMeshing.hpp"

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
	addClass(LaserScan::className, &create<LaserScan> );
	addClass(TriMesh::className, &create<TriMesh> );
	addClass(ScanMeshing::className, &create<ScanMeshing> );

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

