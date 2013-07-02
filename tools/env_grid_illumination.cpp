#include <envire/Core.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/GridIllumination.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_grid_illumination <env_path> <grid_map_id> <grid_band_name> light_x light_y light_z light_d\n"
        << std::endl;
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc != 8)
    {
        std::cerr << "wrong number of arguments" << std::endl;
        usage(1);
    }

    std::string env_path(argv[1]);
    std::string grid_map_id(argv[2]);
    std::string grid_band_name = argv[3];
    base::Vector3d lightPos( 
	    boost::lexical_cast<double>( argv[4] ),
	    boost::lexical_cast<double>( argv[5] ),
	    boost::lexical_cast<double>( argv[6] ) );
    double diameter = 
	    boost::lexical_cast<double>( argv[7] );


    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(env_path));

    envire::GridBase::Ptr input(env->getItem<envire::GridBase>(grid_map_id));
    envire::GridIllumination::Ptr op = new envire::GridIllumination;
    env->attachItem( op.get() );
    op->setOutputBand( grid_band_name );
    op->setLightSource( lightPos, diameter );
    op->addOutput( input.get() );
    op->updateAll();
    env->serialize(env_path);
    std::cout << op->getUniqueId() << std::endl;
    return 0;
}


