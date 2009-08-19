#include "Core.hpp"
#include "LaserScan.hpp"

#include <boost/assign/list_of.hpp>

using namespace std;
using namespace envire;
using namespace boost::assign;

map<std::string, EnvironmentItem::Factory> EnvironmentItem::classMap 
    = map_list_of("envire::FrameNode", FrameNode::create);
