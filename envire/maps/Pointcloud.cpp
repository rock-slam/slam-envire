#include "Core.hpp"
#include "maps/Pointcloud.hpp"
#include "tools/PlyFile.hpp"

#include <fstream>

using namespace envire;

const std::string Pointcloud::className = "envire::Pointcloud";

const std::string Pointcloud::VERTEX_COLOR = "vertex_color";
const std::string Pointcloud::VERTEX_NORMAL = "vertex_normal";
const std::string Pointcloud::VERTEX_VARIANCE = "vertex_variance";
const std::string Pointcloud::VERTEX_ATTRIBUTES = "vertex_attributes";

Pointcloud::Pointcloud()
{
}

Pointcloud::~Pointcloud()
{
}

Pointcloud::Pointcloud(Serialization& so, bool handleMap)
    : Map<3>(so)
{
    so.setClassName(className);

    if(handleMap)
    {
	if( !readPly( getMapFileName(so.getMapPath()) + ".ply" ) )
	    readText( getMapFileName(so.getMapPath()) + ".txt" );
    }
}

void Pointcloud::serialize(Serialization& so)
{
    serialize(so, true);
}

void Pointcloud::serialize(Serialization& so, bool handleMap)
{
    CartesianMap::serialize(so);
    so.setClassName(className);

    if(handleMap)
	writePly( getMapFileName(so.getMapPath()) + ".ply" );
}

Pointcloud* Pointcloud::clone() const
{
    return new Pointcloud(*this);
}

void Pointcloud::set( EnvironmentItem* other )
{
    Pointcloud *p = dynamic_cast<Pointcloud*>( other );
    if( p ) operator=( *p );
}

bool Pointcloud::writePly(const std::string& path)
{
    PlyFile ply(path);
    return ply.serialize( this );
}

bool Pointcloud::readPly(const std::string& path)
{
    PlyFile ply(path);
    return ply.unserialize( this );
}

bool Pointcloud::writeText(const std::string& path)
{
    std::ofstream data(path.c_str());
    if( data.fail() )  
    {
	std::cerr << "Could not open file '" + path + "' for writing." << std::endl;
	return false;
    }

    for(size_t i=0;i<vertices.size();i++)
    {
	data << vertices[i].x() << " " << vertices[i].y() << " " << vertices[i].z() << std::endl;
    }
    
    return true;
}

bool Pointcloud::readText(const std::string& path)
{
    std::ifstream data(path.c_str());
    if( data.fail() )  
    {
	std::cerr << "Could not open file '" + path + "'." << std::endl;
	return false;
    }

    while( !data.eof() )
    {
	double x, y, z;
	data >> x >> y >> z;
	vertices.push_back( Eigen::Vector3d( x,y,z ) );
    }

    return true;
}

Pointcloud* Pointcloud::importCsv(const std::string& path, FrameNode* fm)
{
    Pointcloud* pc = new Pointcloud();
    pc->readText( path );

    Environment* env = fm->getEnvironment();
    env->attachItem(pc);
    pc->setFrameNode(fm);

    return pc;
}

Pointcloud::Extents Pointcloud::getExtents() const
{
    // TODO provide proper extents
    return Extents(); 
}
