#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;

void setHalfCube(envire::TriMesh *mesh)
{
    std::vector<Eigen::Vector3d>& points(mesh->vertices);
    std::vector<envire::TriMesh::vertex_attr>& attr(mesh->getVertexData<envire::TriMesh::vertex_attr>(envire::TriMesh::VERTEX_ATTRIBUTES));
    std::vector<Eigen::Vector3d>& normal(mesh->getVertexData<Eigen::Vector3d>(envire::TriMesh::VERTEX_NORMAL));

    // generate a pointcloud with vertices on 3 adjecent walls of a cube
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
	}
    }
}

void setSineWave(envire::TriMesh *mesh)
{
    std::vector<Eigen::Vector3d>& points(mesh->vertices);
    std::vector<envire::TriMesh::vertex_attr>& attr(mesh->getVertexData<envire::TriMesh::vertex_attr>(envire::TriMesh::VERTEX_ATTRIBUTES));
    std::vector<Eigen::Vector3d>& normal(mesh->getVertexData<Eigen::Vector3d>(envire::TriMesh::VERTEX_NORMAL));

    // generate a pointcloud with vertices on 3 adjecent walls of a cube
    for(int i=-10;i<=10;i++)
    {
	for(int j=-10;j<=10;j++)
	{
	    bool edge = (i == 10 || j == 10 || i == 0 || j == 0 );

	    double x = i*.1;
	    double y = j*.1;
	    double d = sqrt( (x*10)*(x*10) + (y*10)*(y*10) ); 
	    double z = cos( d );

	    Eigen::Vector3d norm( 
		    -cos( atan( -sin(d) ) ),
		    0,
		    sin( atan( -sin(d) ) ) );

	    norm = Eigen::AngleAxisd(atan2(y, x), Eigen::Vector3d::UnitZ()) * norm;

	    points.push_back( Eigen::Vector3d( x, y, z ) );
	    attr.push_back( edge << TriMesh::SCAN_EDGE );
	    normal.push_back(norm);
	}
    }
}

int main( int argc, char* argv[] )
{
    boost::scoped_ptr<Environment> env(new Environment());

    FrameNode *fm1 = new FrameNode();
    FrameNode *fm2 = new FrameNode();
    env->addChild( env->getRootNode(), fm1 );
    env->addChild( env->getRootNode(), fm2 );

    TriMesh* mesh = new TriMesh();
    env->attachItem( mesh );
    setSineWave( mesh );

    TriMesh* mesh2 = new TriMesh();
    env->attachItem( mesh2 );
    setSineWave( mesh2 );

    mesh->setFrameNode( fm1 );
    mesh2->setFrameNode( fm2 );

    std::cout << "------ origin at 0,0" << std::endl;

    fm1->setTransform( 
	    Eigen::Translation3d( 0,0,0 )
	    * Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitX()) );

    fm2->setTransform( 
	    Eigen::Translation3d( 0,0,0 )
	    * Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX()) );

    ICP icp;
    icp.getConfiguration().density = 1;
    icp.getConfiguration().threshold = .5;
    icp.addToModel( mesh );

    icp.align( mesh2, 2, 0.01 );
    
    std::cout << std::endl;
    std::cout << "------- origin at 10,10" << std::endl;

    fm1->setTransform( 
	    Eigen::Translation3d( 10,10,0 )
	    * Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitX()) );

    fm2->setTransform( 
	    Eigen::Translation3d( 10,10,0 )
	    * Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX()) );
    
    icp.align( mesh2, 2, 0.01 );
} 
