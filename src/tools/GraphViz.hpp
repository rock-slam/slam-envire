#ifndef __ENVIRE_TOOLS_GRAPHVIZ__
#define __ENVIRE_TOOLS_GRAPHVIZ__

#include <envire/Core.hpp>
#include <fstream>

namespace envire
{
class GraphViz
{
public:
    void writeToFile( Environment* env, const std::string& outputfile );
};

}

#endif
