#include "GraphViz.hpp"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace envire;

void GraphViz::writeToFile( Environment* env, const std::string& outputfile )
{
    std::ofstream os(outputfile.c_str());

    os << "digraph G {" << std::endl;

    foreach( const Environment::itemListType::value_type& pair, env->items )
    {
	EnvironmentItem* item = pair.second.get();

	std::string classname = item->getClassName();
	// remove any namespaces
	if( classname.find("::") != std::string::npos )
	    classname = classname.substr( classname.rfind("::") + 2 );

	os << "g" << item->getUniqueId() << " "
	    << "[label=\"" << classname << "[" << item->getUniqueId() << "]" << "\"";

	if( dynamic_cast<FrameNode*>(item) )
	    os << ",shape=ellipse,style=filled,fillcolor=lightblue";

	if( dynamic_cast<Operator*>(item) )
	    os << ",shape=octagon,style=filled,fillcolor=lightcoral";

	if( dynamic_cast<Layer*>(item) )
	    os << ",shape=box,style=filled,fillcolor=palegreen";

	os << "]" << std::endl;
    }

    foreach( const Environment::frameNodeTreeType::value_type& pair, env->frameNodeTree )
    {
	os 
	    << "g" << pair.second->getUniqueId() 
	    << " -> g" << pair.first->getUniqueId()
	    << std::endl;
    }

    foreach( const Environment::layerTreeType::value_type& pair, env->layerTree )
    {
	os 
	    << "g" << pair.second->getUniqueId() 
	    << " -> g" << pair.first->getUniqueId()
	    << std::endl;
    }

    foreach( const Environment::operatorGraphType::value_type& pair, env->operatorGraphInput )
    {
	os 
	    << "g" << pair.second->getUniqueId() 
	    << " -> g" << pair.first->getUniqueId()
	    << std::endl;
    }

    foreach( const Environment::operatorGraphType::value_type& pair, env->operatorGraphOutput )
    {
	os 
	    << "g" << pair.first->getUniqueId() 
	    << " -> g" << pair.second->getUniqueId()
	    << std::endl;
    }

    foreach( const Environment::cartesianMapGraphType::value_type& pair, env->cartesianMapGraph )
    {
	os 
	    << "g" << pair.first->getUniqueId() 
	    << " -> g" << pair.second->getUniqueId()
	    << std::endl;
    }

    os << "}" << std::endl;

}
