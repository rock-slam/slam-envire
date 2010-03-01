#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;

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
    Eigen::Transform3d t1( Eigen::Translation3d( 0,0,-0.5 ) );
    t1 *= Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());

    std::vector<Eigen::Vector3d>& points(mesh->vertices);
    std::vector<unsigned int>& attr(mesh->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    std::vector<Eigen::Vector3d>& points2(mesh2->vertices);
    std::vector<unsigned int>& attr2(mesh2->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    // generate a pointcloud with vertices on 3 adjecent walls of a cube
    // generate a pointcloud with vertices at all edges of a cube
    for(int i=0;i<=10;i++)
    {
	for(int j=0;j<=10;j++)
	{
	    bool edge = (i == 10 || j == 10);

	    double x = i*.1;
	    double y = j*.1;

	    points.push_back( Eigen::Vector3d( x, y, 0 ) );
	    points.push_back( Eigen::Vector3d( 0, x, y ) );
	    points.push_back( Eigen::Vector3d( x, 0, y ) );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );

	    points2.push_back( t1*Eigen::Vector3d( x, y, 0 ) );
	    points2.push_back( t1*Eigen::Vector3d( 0, x, y ) );
	    points2.push_back( t1*Eigen::Vector3d( x, 0, y ) );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	}
    }

    ICP icp;
    icp.addToModel( mesh );

    icp.align( mesh2, 10, 0.01 );

} 
