#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_add_grid <env_path> <grid_file> <band_name> [map_type] -frame <frame_id>\n"
        << "       env_add_grid <env_path> <grid_file> <band_name> -map <map_id>\n"
        << "       env_add_grid <env_path> <grid_file> <band_name> [map_type]\n"
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
    std::string map_type;
    if (argc > 4)
    {
        std::string mode = argv[4];
        int next_idx = 5;
        if (mode != "-frame" && mode != "-map")
        {
            // assume this is a map type
            map_type = mode;
            next_idx++;
        }

        if (argc > next_idx)
        {
            if (argc != next_idx + 2)
                usage(1);

            std::string mode = argv[next_idx];
            if (mode ==  "-frame")
                frame_id = boost::lexical_cast<int>(argv[next_idx + 1]);
            else if (mode == "-map")
                map_id = boost::lexical_cast<int>(argv[next_idx + 1]);
        }
    }

    if (frame_id == -1 && map_id == -1)
    {
        if (transform.isApprox(FrameNode::TransformType::Identity()))
            frame_id = env->getRootNode()->getUniqueId();
        else
        {
            FrameNode::Ptr node = new FrameNode(transform);
            env->attachItem(node.get());
            env->getRootNode()->addChild(node.get());
            frame_id = node->getUniqueId();
        }
    }

    if (!map_type.empty())
    {
        if (frame_id == -1)
        {
            std::cerr << "both map type and -map have been specified" << std::endl;
            usage(1);
        }

        GridBase::Ptr target = GridBase::create(map_type,
                input->getWidth(), input->getHeight(),
                input->getScaleX(), input->getScaleY(),
                input->getOffsetX(), input->getOffsetY());
        env->attachItem(target.get());

        envire::FrameNode::Ptr frame = env->getItem<envire::FrameNode>(frame_id);
        target->setFrameNode(frame.get());

        frame_id = -1;
        map_id = target->getUniqueId();
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
    std::cout << map_id << std::endl;
    return 0;
}

