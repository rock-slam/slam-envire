#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"
#include "envire/Grid.hpp"
#include "envire/Projection.hpp"

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
    
    std::list<envire::Operator*> ops = env->getOperators();
    for(std::list<envire::Operator*>::iterator it=ops.begin();it!=ops.end();it++)
    {
	(*it)->updateAll();
	std::cout << "update trimesh" << std::endl;
    }

    // create new grid
    envire::Grid *grid = new envire::Grid(400, 2000, 0.05, 0.05);
    env->attachItem( grid );
    grid->setFrameNode( env->getRootNode() );

    // and operator
    envire::Projection *proj = new envire::Projection();
    env->attachItem( proj );
    proj->addOutput( grid );

    std::list<FrameNode*> fm( env->getChildren( env->getRootNode() ) );
    for(std::list<FrameNode*>::iterator it=fm.begin();it!=fm.end();it++)
    {
	std::list<CartesianMap*> maps( env->getMaps( *it ) );

	for(std::list<CartesianMap*>::iterator mi=maps.begin();mi!=maps.end();mi++)
	{
	    if((*mi)->getClassName() == "envire::TriMesh")
	    {
		std::cout << "adding trimesh to projection" << std::endl;
		proj->addInput( dynamic_cast<TriMesh*>(*mi) );
	    }
	}
    }

    proj->updateAll();

    std::string path(argv[2]);
    so.serialize(env.get(), path);
} 
