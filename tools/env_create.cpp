#include "envire/Core.hpp"
#include <boost/scoped_ptr.hpp>
#include <iostream>

int main( int argc, char* argv[] )
{
    if( argc != 2 ) 
    {
	std::cout << "usage: env_create path" << std::endl;
        std::cout << "  creates an empty envire environment" << std::endl;
	exit(0);
    }
    boost::scoped_ptr<envire::Environment> env(new envire::Environment);
    env->serialize(argv[1]);
}

