#include "Core.hpp"
#include "Pointcloud.hpp"

using namespace envire;

const std::string Pointcloud::className = "envire::Pointcloud";

Pointcloud::Pointcloud()
{
}

Pointcloud::~Pointcloud()
{
}

Pointcloud::Pointcloud(Serialization& so)
    : CartesianMap(so)
{
    so.setClassName(className);
}

void Pointcloud::serialize(Serialization& so)
{
    CartesianMap::serialize(so);
    so.setClassName(className);
}

Pointcloud* Pointcloud::clone() 
{
    return new Pointcloud(*this);
}

void Pointcloud::writeMap(const std::string& path)
{
    // TODO write to ply file
    throw std::runtime_error("not yet implemented");
}

void Pointcloud::readMap(const std::string& path)
{
    // TODO read from ply file
    throw std::runtime_error("not yet implemented");
}

