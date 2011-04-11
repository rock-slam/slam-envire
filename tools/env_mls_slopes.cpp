#include "envire/Core.hpp"
#include "envire/maps/Grid.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/operators/MLSSlope.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_mls_slope input output [mls_id]" << std::endl;
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
    if (argc == 4)
    {
        // An explicit map ID was given
        mls = env->getItem<MLSGrid>(boost::lexical_cast<int>(argv[3]));
        if (!mls)
        {
            std::cerr << "the specified environment has no MLS with an ID of " << argv[3] << std::endl;
            exit(1);
        }
    }
    else
    {
        mls = env->getItem<MLSGrid>();
    }

    boost::intrusive_ptr< envire::Grid<double> > grid = new envire::Grid<double>(
            mls->getWidth(), mls->getHeight(),
            mls->getScaleX(), mls->getScaleY());
    env->attachItem(grid.get());
    grid->setFrameNode(mls->getFrameNode());

    // Create the convertion operator and run it
    envire::MLSSlope *op = new envire::MLSSlope();
    env->attachItem( op );
    op->addInput(mls.get());
    op->addOutput(grid.get());
    op->updateAll();

    // detach the resulting pointcloud from the existing environment, and place
    // into a newly created one.
    Environment env2;
    grid->cloneTo(env2);
    env2.serialize(argv[2]);
} 

