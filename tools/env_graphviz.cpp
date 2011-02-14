#include "envire/tools/GraphViz.hpp"
#include "envire/Core.hpp"
#include "boost/scoped_ptr.hpp"

using namespace envire;

int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_graphviz input output" << std::endl;
	exit(0);
    }

    Serialization so;
    boost::scoped_ptr<Environment> env(so.unserialize( argv[1] ));

    GraphViz gv;
    gv.writeToFile( env.get(), argv[2] );
}
