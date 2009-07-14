#include "Core.hpp"
#include "TriMesh.hpp"

#include <boost/shared_ptr.hpp>

using namespace envire;

TriMesh::TriMesh(FrameNode_Ptr node, std::string const& id) :
    CartesianMap(node, id)
{
}

TriMesh::TriMesh(std::string const& id) :
    CartesianMap(id)
{
}

Layer_Ptr TriMesh::clone(std::string const& id) 
{
    TriMesh* c = new TriMesh(*this);
//    Layer_Ptr clone = Layer_Ptr(new TriMesh(*this));
//   clone->id = id;
    c->id = id;
    Layer_Ptr clone = Layer_Ptr(c);
    return clone;
}


