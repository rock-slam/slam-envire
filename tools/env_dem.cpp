#include <Eigen/Geometry>
#include "icp/icp.hpp"

#include "envire/Core.hpp"
#include "envire/maps/TriMesh.hpp"
#include "envire/operators/ScanMeshing.hpp"
#include "envire/maps/Grids.hpp"
#include "envire/operators/Projection.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_icp input output" << std::endl;
	exit(0);
    }

    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));
    
    env->updateOperators();

    // fix resolution
    const float res = 0.1;

    // add operator
    envire::Projection *proj = new envire::Projection();
    env->attachItem( proj );

    // add input pointclouds
    envire::Pointcloud::Extents extents;
    std::vector<envire::Pointcloud*> meshes = env->getItems<envire::Pointcloud>();
    for(std::vector<envire::Pointcloud*>::iterator it=meshes.begin();it!=meshes.end();it++)
    {
	std::cout << "adding pointcloud to projection" << std::endl;
	proj->addInput( *it );
	extents.extend( (*it)->getExtentsTransformed( *env->getRootNode() ) );
    }

    std::cout << "Grid Extents: " << std::endl
	<< "min: " << extents.min().transpose() << std::endl
	<< "max: " << extents.max().transpose() << std::endl;

    // generate output grid
    envire::FrameNode *fm1 = new envire::FrameNode();
    env->addChild(env->getRootNode(), fm1);
    fm1->setTransform( FrameNode::TransformType(Eigen::Translation3d(extents.min().x(), extents.min().y(), 0)) );

    // create the grid at the right size
    envire::Pointcloud::Extents::VectorType dim = extents.max() - extents.min();
    envire::ElevationGrid *grid = new envire::ElevationGrid(dim.x()/res, dim.y()/res, res, res);
    env->attachItem( grid );
    grid->setFrameNode( fm1 );

    // update to projection and add output
    proj->addOutput( grid );
    proj->updateAll();

    std::string path(argv[2]);
    env->serialize( path );
} 
