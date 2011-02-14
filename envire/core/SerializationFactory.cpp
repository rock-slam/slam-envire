#include "Core.hpp"

#include "maps/LaserScan.hpp"
#include "maps/TriMesh.hpp"
#include "maps/Pointcloud.hpp"
#include "maps/Grids.hpp"
#include "maps/MultiLevelSurfaceGrid.hpp"
#include "operators/ScanMeshing.hpp"
#include "operators/Projection.hpp"
#include "operators/MLSProjection.hpp"

using namespace envire;


std::map<std::string, SerializationFactory::Factory>& SerializationFactory::getMap()
{
    static std::map<std::string, Factory> classMap;
    return classMap;
}

EnvironmentItem* SerializationFactory::createObject( const std::string& className, Serialization& so )
{
    std::map<std::string, Factory> &classMap( getMap() );
    Factory f = NULL;
    if( classMap.count( className ) )
	f = classMap[className];
    else 
    {
	std::string name = className.substr( className.rfind("::") + 2 );
	if( classMap.count( name ) )
	    f = classMap[name];
    }

    if( !f )
    {
	std::cerr << "could not find class of type " << className << std::endl;
	std::cerr << "did you forget to add the ENVIRONMENT_ITEM_DEF macro for this class?" << std::endl;
	throw std::runtime_error("could not find class of type " + className );
    }

    EnvironmentItem* envItem = (*f)(so);
    return envItem;
}

void SerializationFactory::addClass( const std::string& className, Factory f )
{
    getMap()[className] = f;
}

