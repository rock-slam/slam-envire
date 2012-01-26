#include "envire/Core.hpp"
#include "envire/maps/TriMesh.hpp"
#include "boost/scoped_ptr.hpp"

#include <iostream>

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 2 ) 
    {
	std::cout << "usage: env_simplify input" << std::endl;
	exit(0);
    }

    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));

} 

