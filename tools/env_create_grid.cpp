#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/Grid.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <boost/scoped_ptr.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_create_grid <env_path> <type> <width> <height> <scale_x> <scale_y> [<offset_x> <offset_y>] -frame <frame_id> [-bands band1 band2 ...]\n"
        << "       env_create_grid <env_path> <type> <width> <height> <scale_x> <scale_y> [<offset_x> <offset_y>]\n"
        << "       env_create_grid <env_path> <type> -map <map_id>\n"
        << "  creates a new grid map of the specified type (as the envire class name) in the specified environment\n"
        << "\n"
        << "  if the -frame option is given, the new grid is attached to the specified frame node\n"
        << "  if the -bands option is given, the specied bands are generated in the grid\n"
        << "  if the -map option is given, the grid parameters (size, scale, offsets) are copied from it\n";

    std::cerr << "available grid types: ";
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc < 4)
        usage(1);

    int argi = 1;
    std::string env_path(argv[argi++]);
    std::string grid_type(argv[argi++]);

    boost::scoped_ptr<Environment> env(Environment::unserialize(env_path));

    double scale_x = 1, scale_y = 1;
    int width, height;
    double offset_x = 0, offset_y = 0;
    FrameNode* base_frame = env->getRootNode();
    std::vector<std::string> bands;

    if ( !strcmp( argv[argi], "-map" ))
    {
	argi++;
        if (argc != 5)
        {
            std::cerr << "too many or too little arguments provided" << std::endl;
            usage(1);
        }
        std::string map_id(argv[argi++]);
        boost::intrusive_ptr<GridBase> map = env->getItem<GridBase>(map_id);
        if (!map)
        {
            std::cerr << "specified map ID " << map_id << " does not exist" << std::endl;
            return 1;
        }
        scale_x = map->getScaleX();
        scale_y = map->getScaleY();
        offset_x = map->getOffsetX();
        offset_y = map->getOffsetY();
        width  = map->getWidth();
        height = map->getHeight();
        base_frame = map->getFrameNode();
    }
    else
    {
        if (argc < 7)
        {
            std::cerr << "missing arguments" << std::endl;
            usage(1);
        }
        width   = boost::lexical_cast<int>(argv[argi++]);
        height  = boost::lexical_cast<int>(argv[argi++]);
        scale_x = boost::lexical_cast<double>(argv[argi++]);
        scale_y = boost::lexical_cast<double>(argv[argi++]);
        if (argc > 7)
        {
            if (argc < 9)
            {
                std::cerr << "missing arguments" << std::endl;
                usage(1);
            }

            if ( strcmp( argv[argi], "-frame") )
            {
                offset_x = boost::lexical_cast<double>(argv[argi++]);
                offset_y = boost::lexical_cast<double>(argv[argi++]);
            }

            if ( strcmp( argv[argi], "-frame") ) 
	    {
		std::cerr << "expected -frame argument" << std::endl;
		usage(1);
	    }
	    argi++;

            if (argc < 11)
            {
                std::cerr << "too little arguments provided" << std::endl;
                usage(1);
            }

            std::string frame_id(argv[argi++]);
            base_frame = env->getItem<FrameNode>(frame_id).get();
            if (!base_frame)
            {
                std::cerr << "specified frame ID " << frame_id << " does not exist" << std::endl;
                exit(1);
            }

	    if( argi < argc && !strcmp( argv[argi], "-bands" ) )
	    {
		argi++;
		while( argi < argc )
		    bands.push_back( argv[argi++] );
	    }
        }
    }

    std::cerr << "creating new grid of type " << grid_type << " with\n"
        << "  Size in cells: " << width << "x" << height << "\n"
        << "  Scale: " << scale_x << "x" << scale_y << "\n"
        << "  Offset: " << offset_x << "x" << offset_y << "\n"
        << "  Frame ID: " << base_frame->getUniqueId() 
	<< "  bands: ";
    for(std::vector<std::string>::iterator it = bands.begin(); it != bands.end(); it++ )
	std::cerr << *it << " ";
    std::cerr << std::endl;

    GridBase::Ptr grid = GridBase::create(grid_type,
            width, height, scale_x, scale_y, offset_x, offset_y);

    // if there are bands, try to create them
    if( !bands.empty() )
    {
	// try upcasting to banded grid
	envire::BandedGrid *bgrid = dynamic_cast<envire::BandedGrid*>( grid.get() );
	if( bgrid )
	{
	    for(std::vector<std::string>::iterator it = bands.begin(); it != bands.end(); it++ )
	    {
		bgrid->createBand( *it );
	    }
	}
	else
	{
	    std::cerr << "Specified bands, but grid type does not support them" << std::endl;
	    exit(1);
	}
    }

    env->attachItem(grid.get());
    env->setFrameNode(grid.get(), base_frame);
    env->serialize(env_path);
    std::cout << grid->getUniqueId() << std::endl;
    return 0;
}

