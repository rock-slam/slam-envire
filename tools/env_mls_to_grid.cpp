#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSToGrid.hpp>
#include <boost/scoped_ptr.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_mls_to_grid <env_path> <mls_map_id> <grid_map_id> <grid_band_name>\n"
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
    int mls_map_id  = boost::lexical_cast<int>(argv[2]);
    int grid_map_id = boost::lexical_cast<int>(argv[3]);
    std::string grid_band_name = argv[4];

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(env_path));

    envire::MLSGrid::Ptr  input(env->getItem<envire::MLSGrid>(mls_map_id));
    envire::Grid<double>::Ptr output(env->getItem< envire::Grid<double> >(grid_map_id));
    envire::MLSToGrid::Ptr op = new envire::MLSToGrid;
    env->attachItem(op.get());
    op->setInput(input.get());
    op->setOutput(output.get(), grid_band_name);
    op->updateAll();
    env->serialize(env_path);
    std::cout << op->getUniqueId() << std::endl;
    return 0;
}


