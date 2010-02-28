#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;

int main( int argc, char* argv[] )
{
    boost::scoped_ptr<Environment> env(new Environment());

    FrameNode *fm1 = new FrameNode();
    FrameNode *fm2 = new FrameNode();
    env->addChild( env->getRootNode(), fm1 );
    env->addChild( env->getRootNode(), fm2 );

    TriMesh* mesh = new TriMesh();
    env->attachItem( mesh );

    TriMesh* mesh2 = new TriMesh();
    env->attachItem( mesh2 );

    mesh->setFrameNode( fm1 );
    mesh2->setFrameNode( fm2 );

    // initialise a number of transforms
    Eigen::Transform3f t1( Eigen::Translation3f( 0,0,.2 ) );
    t1 *= Eigen::AngleAxisf(0.2, Eigen::Vector3f::UnitX());

    std::vector<Eigen::Vector3f>& points(mesh->vertices);
    std::vector<unsigned int>& attr(mesh->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    std::vector<Eigen::Vector3f>& points2(mesh2->vertices);
    std::vector<unsigned int>& attr2(mesh2->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    // generate a pointcloud with vertices on 3 adjecent walls of a cube
    // generate a pointcloud with vertices at all edges of a cube
    for(int i=0;i<10;i++)
    {
	for(int j=0;j<10;j++)
	{
	    bool edge = (i == 10 || j == 10);

	    points.push_back( Eigen::Vector3f( i, j, 0 ) );
	    points.push_back( Eigen::Vector3f( 0, i, j ) );
	    points.push_back( Eigen::Vector3f( i, 0, j ) );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );

	    points2.push_back( t1*Eigen::Vector3f( i, j, 0 ) );
	    points2.push_back( t1*Eigen::Vector3f( 0, i, j ) );
	    points2.push_back( t1*Eigen::Vector3f( i, 0, j ) );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	}
    }

    ICP icp;
    icp.addToModel( mesh );

    icp.align( mesh2, 5, 1.0 );
} 
