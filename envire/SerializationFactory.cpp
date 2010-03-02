#include "Core.hpp"

#include "LaserScan.hpp"
#include "TriMesh.hpp"
#include "ScanMeshing.hpp"

#include <boost/assign/list_of.hpp>

using namespace boost::assign;
using namespace envire;

template<class T> EnvironmentItem* create(Serialization &so) 
{
    T* o = new T(so);
    return o;
}

//
// Map of all classes, that the serialization class knows about
// all classes that can be instantiated need to be added here.
// 
std::map<std::string, Serialization::Factory> Serialization::classMap 
    = map_list_of
	(FrameNode::className, &create<FrameNode> )
	(LaserScan::className, &create<LaserScan> )
	(TriMesh::className, &create<TriMesh> )
	(ScanMeshing::className, &create<ScanMeshing> );
