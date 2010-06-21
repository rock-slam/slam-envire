#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;

     
int main( int argc, char* argv[] )
{
    if( argc < 2 ) 
    {
	std::cout << "usage: env_simplify input" << std::endl;
	exit(0);
    }

    Serialization so;
    boost::scoped_ptr<Environment> env(so.unserialize( argv[1] ));

} 

