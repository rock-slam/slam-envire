#include "envire/Core.hpp"
#include "boost/scoped_ptr.hpp"

#include <iostream>

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
        std::cout << "usage: env_delete <path_to_env> <item_id> [<item_id>]\n"
            << "  Removes the items with the specified item IDs from the environment\n"
            << std::endl;
        exit(1);
    }

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(argv[1]));

    for (int i = 2; i < argc; ++i)
    {
        int item_id = boost::lexical_cast<int>(argv[i]);
        EnvironmentItem* item = env->getItem(item_id).get();
        if (!item)
            std::cerr << "cannot find an item with ID " << item_id << std::endl;
        else
            env->detachItem(item, true);

    }

    env->serialize(argv[1]);
}

