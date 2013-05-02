#include "envire/Core.hpp"
#include "envire/maps/Grid.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/maps/Pointcloud.hpp"
#include "envire/operators/MLSSlope.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 2 ) 
    {
	std::cout << "usage: env_mls_slope input [mls_id]" << std::endl;
        std::cout << "  generates an maximum slope map based on MLS data" << std::endl;
        std::cout << "  the generated map will have the same width, height and cell size than the MLS" << std::endl;
        std::cout << std::endl;
        std::cout << "If mls_id is given, it is the map ID of the MLS that is to be processed." << std::endl;
        std::cout << "Otherwise, an MLSGrid map is searched for in the environment. An error is generated if there are more than one, or none." << std::endl;
	exit(0);
    }
    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));

    env->updateOperators();

    boost::intrusive_ptr<MLSGrid> mls;
    if (argc == 3)
    {
        // An explicit map ID was given
        mls = env->getItem<MLSGrid>(argv[2]);
        if (!mls)
        {
            std::cerr << "the specified environment has no MLS with an ID of " << argv[2] << std::endl;
            exit(1);
        }
    }
    else
    {
        mls = env->getItem<MLSGrid>();
    }

    envire::Pointcloud *pc = new Pointcloud();
    env->setFrameNode( pc, mls->getFrameNode() );

    for(size_t m=0;m<mls->getWidth();m++)
    {
	for(size_t n=0;n<mls->getHeight();n++)
	{
	    for( MLSGrid::iterator cit = mls->beginCell(m,n); cit != mls->endCell(); cit++ )
	    {
		MLSGrid::SurfacePatch p( *cit );

		// get 3d position of gridcell
		Eigen::Vector3d pos;
		pos << mls->fromGrid( GridBase::Position( m, n ) ), p.mean;

		pc->vertices.push_back( pos );
	    }
	}
    }

    env->serialize(argv[1]);
} 

