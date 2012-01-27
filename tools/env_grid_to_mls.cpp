#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/GridFloatToMLS.hpp>
#include <boost/scoped_ptr.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_grid_to_mls <env_path> <grid_map_id> <grid_band_name> <mls_map_id>\n"
        << std::endl;
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc != 5)
    {
        std::cerr << "wrong number of arguments" << std::endl;
        usage(1);
    }

    std::string env_path(argv[1]);
    int grid_map_id = boost::lexical_cast<int>(argv[2]);
    std::string grid_band_name = argv[3];
    int mls_map_id  = boost::lexical_cast<int>(argv[4]);

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(env_path));

    envire::GridBase::Ptr input(env->getItem<envire::GridBase>(grid_map_id));
    envire::MLSGrid::Ptr  output(env->getItem<envire::MLSGrid>(mls_map_id));
    envire::GridFloatToMLS::Ptr op = new envire::GridFloatToMLS;
    env->attachItem(op.get());
    op->setInput(input.get(), grid_band_name);
    op->setOutput(output.get());
    env->updateOperators();
    env->serialize(env_path);
    return 0;
}


