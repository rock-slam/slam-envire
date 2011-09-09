#include "GraphViz.hpp"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
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

	os << "g" << item->getUniqueId() << " ";
	std::string name = (boost::format("%s [%i]") % classname % item->getUniqueId()).str();
	if( !item->getLabel().empty() )
	    name = item->getLabel() + "\\n" + name;

	if( dynamic_cast<FrameNode*>(item) )
	{
	    FrameNode* fn = dynamic_cast<FrameNode*>(item);
	    Eigen::Vector3d t = fn->getTransform().translation();
	    Eigen::Vector3d r = fn->getTransform().linear().eulerAngles(2,1,0) / M_PI * 180;

	    os 
		<< "[label=\"" << name <<  
		boost::format("\\nt: (%.1f %.1f %.1f)\\nr: (%.1f %.1f %.1f)") % t.x() % t.y() % t.z() % r.z() % r.y() % r.x() 
		<< "\""
		<< ",shape=ellipse,style=filled,fillcolor=lightblue";
	}

	if( dynamic_cast<Operator*>(item) )
	    os 
		<< "[label=\"" << name << "\""
		<< ",shape=octagon,style=filled,fillcolor=lightcoral";

	if( dynamic_cast<Layer*>(item) )
	    os 
		<< "[label=\"" << name << "\""
		<< ",shape=box,style=filled,fillcolor=palegreen";

	os << "]" << std::endl;
    }

    os << "# framenodetree" << std::endl;
    foreach( const Environment::frameNodeTreeType::value_type& pair, env->frameNodeTree )
    {
	os 
	    << "g" << pair.first->getUniqueId() 
	    << " -> g" << pair.second->getUniqueId()
	    << std::endl;
    }

    os << "# layertree" << std::endl;
    foreach( const Environment::layerTreeType::value_type& pair, env->layerTree )
    {
	os 
	    << "g" << pair.second->getUniqueId() 
	    << " -> g" << pair.first->getUniqueId()
	    << " [style=dotted]"
	    << std::endl;
    }

    os << "# operatorGraphInput" << std::endl;
    foreach( const Environment::operatorGraphType::value_type& pair, env->operatorGraphInput )
    {
	os 
	    << "g" << pair.second->getUniqueId() 
	    << " -> g" << pair.first->getUniqueId()
	    << std::endl;
    }

    os << "# operatorGraphOutput" << std::endl;
    foreach( const Environment::operatorGraphType::value_type& pair, env->operatorGraphOutput )
    {
	os 
	    << "g" << pair.first->getUniqueId() 
	    << " -> g" << pair.second->getUniqueId()
	    << std::endl;
    }

    os << "# cartesianMapGraph" << std::endl;
    foreach( const Environment::cartesianMapGraphType::value_type& pair, env->cartesianMapGraph )
    {
	os 
	    << "g" << pair.first->getUniqueId() 
	    << " -> g" << pair.second->getUniqueId()
	    << " [shape=dot]"
	    << std::endl;
    }

    os << "}" << std::endl;

}
