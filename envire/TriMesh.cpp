#include "Core.hpp"
#include "TriMesh.hpp"

#include <boost/shared_ptr.hpp>

using namespace envire;

const std::string TriMesh::className = "envire::TriMesh";

TriMesh::TriMesh()
{
}

TriMesh::TriMesh(Serialization& so)
    : CartesianMap(so)
{
    so.setClassName(className);
}

void TriMesh::serialize(Serialization& so)
{
    CartesianMap::serialize(so);
    so.setClassName(className);
}

TriMesh* TriMesh::clone() 
{
    return new TriMesh(*this);
}


