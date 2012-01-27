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
    std::cerr << "usage: env_create_grid <env_path> <type> <width> <height> <scale_x> <scale_y> [<offset_x> <offset_y>] -frame <frame_id>\n"
        << "       env_create_grid <env_path> <type> <width> <height> <scale_x> <scale_y> [<offset_x> <offset_y>]\n"
        << "       env_create_grid <env_path> <type> -map <map_id>\n"
        << "  creates a new grid map of the specified type (as the envire class name) in the specified environment\n"
        << "\n"
        << "  if the -frame option is given, the new grid is attached to the specified frame node\n"
        << "  if the -map option is given, the grid parameters (size, scale, offsets) are copied from it\n";

    std::cerr << "available grid types: ";
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc < 4)
        usage(1);

    std::string env_path(argv[1]);
    std::string grid_type(argv[2]);

    boost::scoped_ptr<Environment> env(Environment::unserialize(env_path));

    double scale_x = 1, scale_y = 1;
    int width, height;
    double offset_x = 0, offset_y = 0;
    FrameNode* base_frame = env->getRootNode();

    std::string next_arg(argv[3]);
    if (next_arg == "-map")
    {
        if (argc != 5)
        {
            std::cerr << "too many or too little arguments provided" << std::endl;
            usage(1);
        }
        int map_id = boost::lexical_cast<int>(argv[4]);
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
        width   = boost::lexical_cast<int>(argv[3]);
        height  = boost::lexical_cast<int>(argv[4]);
        scale_x = boost::lexical_cast<int>(argv[5]);
        scale_y = boost::lexical_cast<int>(argv[6]);
        if (argc > 7)
        {
            if (argc < 9)
            {
                std::cerr << "missing arguments" << std::endl;
                usage(1);
            }

            next_arg = argv[7];
            if (next_arg != "-frame")
            {
                offset_x = boost::lexical_cast<int>(argv[7]);
                offset_y = boost::lexical_cast<int>(argv[8]);
                if (argc > 9) next_arg = argv[9];
            }

            if (next_arg != "-frame") usage(1);

            if (argc != 11)
            {
                std::cerr << "too many or too little arguments provided" << std::endl;
                usage(1);
            }

            int frame_id = boost::lexical_cast<int>(argv[10]);
            base_frame = env->getItem<FrameNode>(frame_id).get();
            if (!base_frame)
            {
                std::cerr << "specified frame ID " << frame_id << " does not exist" << std::endl;
                exit(1);
            }
        }
    }

    std::cerr << "creating new grid of type " << grid_type << " with\n"
        << "  Size in cells: " << width << "x" << height << "\n"
        << "  Scale: " << scale_x << "x" << scale_y << "\n"
        << "  Offset: " << offset_x << "x" << offset_y << "\n"
        << "  Frame ID: " << base_frame->getUniqueId() << std::endl;

    GridBase::Ptr grid = GridBase::create(grid_type,
            width, height, scale_x, scale_y, offset_x, offset_y);
    env->attachItem(grid.get());
    env->setFrameNode(grid.get(), base_frame);
    env->serialize(env_path);
    std::cout << grid->getUniqueId() << std::endl;
    return 0;
}

