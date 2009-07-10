#include "Core.hpp"
#include "TriMesh.hpp"

#include <boost/shared_ptr.hpp>

using namespace envire;

TriMesh::TriMesh(FrameNode_Ptr node, std::string const& id) :
    CartesianMap(node, id)
{
}

TriMesh::TriMesh(FrameNode_Ptr node, Operator_Ptr generator, std::string const& id) :
    CartesianMap(node, id)
{
}

Layer_Ptr TriMesh::clone(std::string const& id) 
{
    return Layer_Ptr(new TriMesh(*this));
}


