#include <Eigen/Geometry>
#include "icp/icp.hpp"

#include "envire/Core.hpp"
#include "envire/maps/TriMesh.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/operators/MLSProjection.hpp"

#include "boost/scoped_ptr.hpp"
#include <sstream>

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_mls input output [resolution] [variance] [gap_size] [patch_thickness] [\"min_x min_y max_x max_y\"]" << std::endl;
	exit(0);
    }
    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));
    
    env->updateOperators();

    // create new grid
    envire::FrameNode *fm1 = new envire::FrameNode();
    env->addChild(env->getRootNode(), fm1);
    
    // and operator
    envire::MLSProjection *proj = new envire::MLSProjection();
    proj->useUncertainty( false );
    env->attachItem( proj );

    double res = 0.05;
    if( argc >= 4 )
	res = boost::lexical_cast<double>( argv[3] );

    double var = res;
    if( argc >= 5 )
	var = boost::lexical_cast<double>( argv[4] );

    double gapSize = 0.5;
    if( argc >= 6 )
	gapSize = boost::lexical_cast<double>( argv[5] );

    double patchThickness = res;
    if( argc >= 7 )
	patchThickness = boost::lexical_cast<double>( argv[6] );

    envire::Pointcloud::Extents extents;
    string str_extends;
    if( argc >= 8 )
    {
	str_extends = argv[7];
	std::istringstream iss( str_extends );
	iss >> extents.min().x()
	    >> extents.min().y()
	    >> extents.max().x()
	    >> extents.max().y();
	extents.min().z() = 0.0;
	extents.max().z() = 1.0;
    }

    std::vector<envire::Pointcloud*> meshes = env->getItems<envire::Pointcloud>();
    for(std::vector<envire::Pointcloud*>::iterator it=meshes.begin();it!=meshes.end();it++)
    {
	envire::Pointcloud *pc = *it;
	std::cout << "adding pointcloud to projection" << std::endl;
	if( !pc->hasData( Pointcloud::VERTEX_VARIANCE ) )
	{
	    std::cout << "no point variances found. setting all values to " << var << std::endl;
	    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
	    uncertainty.resize( pc->vertices.size(), var );
	}
	proj->addInput( pc );

	if( str_extends.empty() )
	{
	    // when no extents are given, extract them from the pointcloud

	    // rotate the extents to the grid frame, and extend the local grid 
	    // whith the extents corner
	    Eigen::Affine3d pc2grid = env->relativeTransform( pc->getFrameNode(), fm1 );
	    envire::Pointcloud::Extents pc_extents = pc->getExtents();
	    std::vector<Eigen::Vector3d> corners;
	    // go through all the permutations to get the corners
	    for( int i=0; i<8; i++ )
	    {
		Eigen::Vector3d corner;
		for( int j=0; j<3; j++ )
		    corner[j] = (i>>j)&1 ? pc_extents.min()[j] : pc_extents.max()[j];
		corners.push_back( corner );
	    }

	    for( std::vector<Eigen::Vector3d>::iterator it = corners.begin(); it != corners.end(); it++ )
		extents.extend( pc2grid * (*it) );
	}
    }
    if( extents.isNull() )
    {
	std::cout << "Extents for pointclouds is zero." << std::endl;
	exit(0);
    }

    fm1->setTransform( Transform(Eigen::Translation3d(extents.min().x(), extents.min().y(), 0)) );
    std::cout << "MLSGrid Extents: " << std::endl
    << "min: " << extents.min().transpose() << std::endl
    << "max: " << extents.max().transpose() << std::endl;

    // create the grid at the right size
    envire::Pointcloud::Extents::VectorType dim = extents.max() - extents.min();
    envire::MultiLevelSurfaceGrid *grid = new envire::MultiLevelSurfaceGrid(dim.x()/res, dim.y()/res, res, res);
    env->attachItem( grid );
    grid->setFrameNode( fm1 );
    grid->setGapSize( gapSize );
    grid->setHorizontalPatchThickness( patchThickness );

    proj->addOutput( grid );
    proj->updateAll();

    // detach the resulting pointcloud from the existing environment, and place
    // into a newly created one.
    boost::scoped_ptr<Environment> env2( new Environment() );
    envire::EnvironmentItem::Ptr gridPtr = env->detachItem( grid );
    envire::EnvironmentItem::Ptr fm1Ptr = env->detachItem( fm1 );

    env2->attachItem( fm1 );
    env2->addChild( env2->getRootNode(), fm1 );
    env2->attachItem( grid );
    env2->setFrameNode( grid, fm1 );

    std::string path(argv[2]);
    env2->serialize(path);
} 
