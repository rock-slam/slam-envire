#include "Core.hpp"
#include "Pointcloud.hpp"

#include <fstream>

using namespace envire;

const std::string Pointcloud::className = "envire::Pointcloud";

const std::string Pointcloud::VERTEX_COLOR = "vertex_color";
const std::string Pointcloud::VERTEX_NORMAL = "vertex_normal";
const std::string Pointcloud::VERTEX_ATTRIBUTES = "vertex_attributes";

Pointcloud::Pointcloud()
{
}

Pointcloud::~Pointcloud()
{
}

Pointcloud::Pointcloud(Serialization& so, bool handleMap)
    : CartesianMap(so)
{
    so.setClassName(className);

    if(handleMap)
	readMap( getMapFileName(so.getMapPath()) + ".txt" );
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
	writeMap( getMapFileName(so.getMapPath()) + ".txt" );
}

Pointcloud* Pointcloud::clone() 
{
    return new Pointcloud(*this);
}

void Pointcloud::writeMap(const std::string& path)
{
    std::ofstream data(path.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + path + "' for writing.");
    }

    for(int i=0;i<vertices.size();i++)
    {
	data << vertices[i].x() << " " << vertices[i].y() << " " << vertices[i].z() << std::endl;
    }
}

void Pointcloud::readMap(const std::string& path)
{
    std::ifstream data(path.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + path + "'.");
    }

    while( !data.eof() )
    {
	double x, y, z;
	data >> x >> y >> z;
	vertices.push_back( Eigen::Vector3d( x,y,z ) );
    }
}

Pointcloud* Pointcloud::importCsv(const std::string& path, FrameNode* fm)
{
    Pointcloud* pc = new Pointcloud();
    pc->readMap( path );

    Environment* env = fm->getEnvironment();
    env->attachItem(pc);
    pc->setFrameNode(fm);

    return pc;
}

