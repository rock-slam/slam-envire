#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "envire/MultiLevelSurfaceGrid.hpp"
#include "envire/MLSProjection.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_mls input output" << std::endl;
	exit(0);
    }

    Serialization so;
    boost::scoped_ptr<Environment> env(so.unserialize( argv[1] ));
    
    env->updateOperators();

    // create new grid
    envire::FrameNode *fm1 = new envire::FrameNode();
    env->addChild(env->getRootNode(), fm1);
    fm1->setTransform( FrameNode::TransformType(Eigen::Translation3d(-9,-3,0)*Eigen::AngleAxisd(-0.15*M_PI, Eigen::Vector3d::UnitZ())) );
    
    double res = 0.05;
    envire::MultiLevelSurfaceGrid *grid = new envire::MultiLevelSurfaceGrid(12/res, 85/res, res, res);
    env->attachItem( grid );
    grid->setFrameNode( fm1 );

    // and operator
    envire::MLSProjection *proj = new envire::MLSProjection();
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
