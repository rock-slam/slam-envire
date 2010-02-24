#include <Eigen/Geometry>
#include "icp.hpp"

BOOST_AUTO_TEST_CASE( half_cube ) 
{
    Environment* env = new Environment(); 

    TriMesh* mesh = new TriMesh();
    env->attachItem( mesh );

    TriMesh* mesh2 = new TriMesh();
    env->attachItem( mesh2 );

    // initialise a number of transforms
    Eigen::Transform3f t1 = Eigen::Translation3f( 0,0,.2 );
    t *= Eigen::AngleAxisf(0.2, Vector3f::UnitX());

    std::vector<Eigen::Vector3f>& points2(mesh2->points);
    std::vector<unsigned int>& attrs2(mesh2->getData(TriMesh::VERTEX_ATTRIBUTES));

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
	    attr.push_back( edge << TriMesh::EDGE );
	    attr.push_back( edge << TriMesh::EDGE );
	    attr.push_back( edge << TriMesh::EDGE );

	    points2.push_back( t*Eigen::Vector3f( i, j, 0 ) );
	    points2.push_back( t*Eigen::Vector3f( 0, i, j ) );
	    points2.push_back( t*Eigen::Vector3f( i, 0, j ) );
	    attr2.push_back( edge << TriMesh::EDGE );
	    attr2.push_back( edge << TriMesh::EDGE );
	    attr2.push_back( edge << TriMesh::EDGE );
	}
    }

    ICP icp;
    icp.addToModel( mesh );

    icp.align( mesh2, 5, 1.0 );

    delete env;
} 
