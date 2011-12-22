#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "envire/SurfaceReconstruction.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_simplify input output" << std::endl;
	exit(0);
    }

    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));
    
    // grab first pointcloud in file
    std::vector<envire::Pointcloud*> pcs = env->getItems<envire::Pointcloud>();
    if( pcs.size() < 1 )
    {
	std::cout << "no pointcloud in environment" << std::endl;
	exit(0);
    }
    envire::Pointcloud *pc = *pcs.begin();

    // and simplified pointcloud
    envire::TriMesh *mesh = new envire::TriMesh();
    env->attachItem( mesh );
    mesh->setFrameNode( pc->getFrameNode() );

    envire::SurfaceReconstruction *reconstruct = new envire::SurfaceReconstruction();
    env->attachItem( reconstruct );
    reconstruct->addInput( pc );
    reconstruct->addOutput( mesh );

    reconstruct->updateAll();

    std::cout << "reconstructed pointcloud to " << mesh->vertices.size() << " points and " << mesh->faces.size() << " faces " << std::endl;

    // detach the resulting pointcloud from the existing environment, and place
    // into a newly created one.
    boost::scoped_ptr<Environment> env2( new Environment() );
    env->detachItem( mesh );

    env2->attachItem( mesh );
    env2->setFrameNode( mesh, env2->getRootNode() );

    std::string path(argv[2]);
    so.serialize(env2.get(), path);
} 

