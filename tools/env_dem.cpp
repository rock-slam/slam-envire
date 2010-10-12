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

    Serialization so;
    boost::scoped_ptr<Environment> env(so.unserialize( argv[1] ));
    
    env->updateOperators();

    // create new grid
    envire::FrameNode *fm1 = new envire::FrameNode();
    env->addChild(env->getRootNode(), fm1);
    fm1->setTransform( FrameNode::TransformType(Eigen::Translation3d(-9,-3,-2)*Eigen::AngleAxisd(-0.15*M_PI, Eigen::Vector3d::UnitZ())) );
    
    double res = 0.10;
    envire::ElevationGrid *grid = new envire::ElevationGrid(12/res, 85/res, res, res);
    env->attachItem( grid );
    grid->setFrameNode( fm1 );

    // and operator
    envire::Projection *proj = new envire::Projection();
    env->attachItem( proj );
    proj->addOutput( grid );

    std::vector<envire::Pointcloud*> meshes = env->getItems<envire::Pointcloud>();
    for(std::vector<envire::Pointcloud*>::iterator it=meshes.begin();it!=meshes.end();it++)
    {
	std::cout << "adding pointcloud to projection" << std::endl;
	proj->addInput( *it );
    }

    proj->updateAll();

    std::string path(argv[2]);
    so.serialize(env.get(), path);
} 
