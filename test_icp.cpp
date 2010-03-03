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
    Eigen::Transform3d t1( Eigen::Translation3d( 0,0,-0.3 ) );
    t1 *= Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    std::vector<Eigen::Vector3d>& points(mesh->vertices);
    std::vector<envire::TriMesh::vertex_attr>& attr(mesh->getData<envire::TriMesh::vertex_attr>(envire::TriMesh::VERTEX_ATTRIBUTES));
    std::vector<Eigen::Vector3d>& normal(mesh->getData<Eigen::Vector3d>(envire::TriMesh::VERTEX_NORMAL));

    std::vector<Eigen::Vector3d>& points2(mesh2->vertices);
    std::vector<envire::TriMesh::vertex_attr>& attr2(mesh2->getData<envire::TriMesh::vertex_attr>(envire::TriMesh::VERTEX_ATTRIBUTES));
    std::vector<Eigen::Vector3d>& normal2(mesh2->getData<Eigen::Vector3d>(envire::TriMesh::VERTEX_NORMAL));

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
	    normal.push_back( Eigen::Vector3d( 0, 0, 1 ) );
	    normal.push_back( Eigen::Vector3d( 1, 0, 0 ) );
	    normal.push_back( Eigen::Vector3d( 0, 1, 0 ) );

	    points2.push_back( t1*Eigen::Vector3d( x, y, 0 ) );
	    points2.push_back( t1*Eigen::Vector3d( 0, x, y ) );
	    points2.push_back( t1*Eigen::Vector3d( x, 0, y ) );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    attr2.push_back( edge << TriMesh::SCAN_EDGE );
	    normal2.push_back( Eigen::Vector3d( 0, 0, 1 ) );
	    normal2.push_back( Eigen::Vector3d( 1, 0, 0 ) );
	    normal2.push_back( Eigen::Vector3d( 0, 1, 0 ) );
	}
    }

    ICP icp;
    icp.addToModel( mesh );

    icp.align( mesh2, 8, 0.01 );

} 
