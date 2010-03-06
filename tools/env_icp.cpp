#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"

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
    
    // go through the environment and convert LaserScans to 
    // this is making a lot of assumption, like
    // the root framenode has a number of direct children,
    // to which the LaserScans are attached.
    // there is only one scan per FrameNode, if there is more,
    // it is assumed, that TriMeshmaps have already been created

    ICP icp;

    std::list<FrameNode*> fm( env->getChildren( env->getRootNode() ) );
    for(std::list<FrameNode*>::iterator it=fm.begin();it!=fm.end();it++)
    {
	std::list<CartesianMap*> maps( env->getMaps( *it ) );
	if( maps.size() == 1 ) 
	{
	    std::cout << "create and add trimesh map to icp" << std::endl;

	    // create a TriMesh Layer and attach it to the root Node.
	    TriMesh* mesh = new TriMesh();
	    env->attachItem( mesh );
	    env->setFrameNode( mesh, *it );

	    // set up a meshing operator on the output mesh. Add then an input
	    // and parametrize the meshing operation. 
	    ScanMeshing* mop = new ScanMeshing();
	    env->attachItem( mop );

	    mop->setMaxEdgeLength(0.5);

	    mop->addInput( dynamic_cast<envire::LaserScan*>(*(maps.begin())) );
	    mop->addOutput(mesh);

	    mop->updateAll();

	    icp.addToModel( mesh );
	}

	for(std::list<CartesianMap*>::iterator mi=maps.begin();mi!=maps.end();mi++)
	{
	    if((*mi)->getClassName() == "envire::TriMesh")
	    {
		std::cout << "adding existing trimesh to icp" << std::endl;
		icp.addToModel( dynamic_cast<TriMesh*>(*mi) );
	    }
	}
    }

    icp.align( 5, 0.01 );

    std::string path(argv[2]);
    so.serialize(env.get(), path);
} 
