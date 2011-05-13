#include "envire/Core.hpp"
#include "boost/scoped_ptr.hpp"

#include <iostream>

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc != 2 ) 
    {
        std::cout << "usage: env_update <path_to_env>\n"
            << "  loads the specified environment and update all the generated maps\n"
            << "  (i.e. runs all operators)\n"
            << std::endl;
        exit(1);
    }

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(argv[1]));
    env->updateOperators();
    env->serialize(argv[1]);
}

