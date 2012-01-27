#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_add_grid <env_path> <grid_file> <band_name> -frame <frame_id>\n"
        << "       env_add_grid <env_path> <grid_file> <band_name> -map <map_id>\n"
        << "       env_add_grid <env_path> <grid_file> <band_name>\n"
        << "  adds the specified file (readable by GDAL as a raster band) to the environment\n"
        << "  and saves the result.\n"
        << "\n"
        << "  if the -frame option is given, a new map is created, that gets attached to the specified frame node\n"
        << "  if the -map option is given, the grid data is added as a new band in an existing map\n"
        << "  finally, if none is given, a new map is created and added to the environment's root frame"
        << "\n"
        << "  files with multiple bands are not supported yet\n"
        << std::endl;
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc < 4 || argc > 6 )
        usage(1);

    std::string env_path(argv[1]);
    std::string grid_file(argv[2]);
    std::string target_band(argv[3]);

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(env_path));
    envire::GridBase::Ptr input;
    FrameNode::TransformType transform;
    boost::tie(input, transform) = GridBase::readGridFromGdal(grid_file, target_band);

    int frame_id = -1, map_id = -1;
    if (argc > 4)
    {
        if (argc != 6)
            usage();

        std::string mode = argv[4];
        if (mode ==  "-frame")
            frame_id = boost::lexical_cast<int>(argv[5]);
        else if (mode == "-map")
            map_id = boost::lexical_cast<int>(argv[5]);
    }
    else if (transform.isApprox(FrameNode::TransformType::Identity()))
        frame_id = env->getRootNode()->getUniqueId();
    else
    {
        FrameNode::Ptr node = new FrameNode(transform);
        env->attachItem(node.get());
        env->getRootNode()->addChild(node.get());
        frame_id = node->getUniqueId();
    }

    if (frame_id != -1)
    {
        envire::FrameNode::Ptr frame = env->getItem<envire::FrameNode>(frame_id);
        env->attachItem(input.get());
        input->setFrameNode(frame.get());
        map_id = input->getUniqueId();
    }
    else
    {
        envire::GridBase::Ptr grid = env->getItem<envire::GridBase>(map_id);
        grid->copyBandFrom(*input, target_band);
    }

    env->serialize(env_path);
    std::cout << std::endl << map_id << std::endl;
    return 0;
}

