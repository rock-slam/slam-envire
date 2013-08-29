#include "Core.hpp"
#include "TriMesh.hpp"

#include <stdexcept>

using namespace envire;

ENVIRONMENT_ITEM_DEF( TriMesh )

TriMesh::TriMesh()
{
}

void TriMesh::serialize(Serialization& so)
{
    Pointcloud::serialize(so, false);

    writePly( getMapFileName() + ".ply" , so.getBinaryOutputStream(getMapFileName() + ".ply"));
}

void TriMesh::unserialize(Serialization& so)
{
    Pointcloud::unserialize(so, false);
    
    readPly( getMapFileName() + ".ply", so.getBinaryInputStream(getMapFileName() + ".ply") );
}

void TriMesh::calcVertexNormals()
{
    // calculate the Triangle normals first
    std::vector<Eigen::Vector3d>& point_normal(getVertexData<Eigen::Vector3d>(TriMesh::VERTEX_NORMAL));
    point_normal.resize( vertices.size(), Eigen::Vector3d::Zero() );
    std::fill( point_normal.begin(), point_normal.end(), Eigen::Vector3d::Zero() );

    // go through all the faces and compute the normals 
    for(size_t i=0;i<faces.size();i++)
    {
	size_t tri[3] = {
            static_cast<size_t>(faces[i].get<0>()),
            static_cast<size_t>(faces[i].get<1>()),
            static_cast<size_t>(faces[i].get<2>())
        };
	Eigen::Vector3d a[3];
	for(int n=0;n<3;n++)
	    a[n] = vertices[ tri[n] ];

	// get the surface normal
	Eigen::Vector3d normal = (a[0]-a[1]).cross(a[2]-a[1]).normalized();

	for(int n=0;n<3;n++)
	    point_normal[tri[n]] = normal;
    }
    
    // normalize the normals and set vertices without normals to defined value
    for(size_t i=0;i<point_normal.size();i++)
    {
	if( point_normal[i].isApprox( Eigen::Vector3d::Zero(), 1e-5 ) )
	{
	    point_normal[i] = Eigen::Vector3d::Zero();
	}
	else 
	{
	    point_normal[i].normalize();
	}
    }
}

