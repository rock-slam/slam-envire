#include "EnvireEventListener.hpp"
#include <envire/maps/LaserScan.hpp>
#include "LaserScanVisualization.hpp"
#include <osg/Node>
#include <algorithm>
#include <osg/Group>

namespace vizkit {
    
    
EnvireEventListener::EnvireEventListener(nodeCallback add, nodeCallback remove)
    : osgAddNode(add), osgRemoveNode(remove)
{    
//	std::cout << "listener connected" << std::endl;
}

osg::Group* EnvireEventListener::getNodeForItem(envire::EnvironmentItem* item)
{
    if(environmentToNode.count(item))
	return environmentToNode[item];
    
    return 0;
}

void EnvireEventListener::setRootNode(envire::FrameNode* root)
{
    if(environmentToNode.count(root) == 0)
	itemAttached(root);
    
    osgAddNode( environmentToNode[root] );
}

void EnvireEventListener::removeRootNode(envire::FrameNode* root)
{
    if(environmentToNode.count(root))
	osgRemoveNode( environmentToNode[root] );
}

void EnvireEventListener::addVisualizer ( EnvironmentItemVisualizer* viz )
{
    visualizers.push_back(viz);
}

void EnvireEventListener::removeVisualizer ( EnvironmentItemVisualizer* viz )
{
    std::vector<EnvironmentItemVisualizer *>::iterator item = std::find(visualizers.begin(), visualizers.end(), viz);
    
    visualizers.erase(item);
    
    //TODO recalculate nodes
}
void EnvireEventListener::addChild(envire::EnvironmentItem* parent, envire::EnvironmentItem* child)
{
    if( !(environmentToNode.count(parent) && environmentToNode.count(child)) )
    {
	// std::cout << "missing item: skipped adding relationship" << std::endl;
	return;
    }

    //be shure child and parent where attached
    assert(environmentToNode.count(parent));
    assert(environmentToNode.count(child));

    //std::cout << "Connection Parent:"<< parent->getUniqueId() << " Child:"<< child->getUniqueId() << std::endl;

    {
	//as we should know visualizers for every node we can simply 
	//map to osg nodes
	environmentToNode[parent]->addChild(environmentToNode[child]);        
    }
}

void EnvireEventListener::removeChild(envire::EnvironmentItem* parent, envire::EnvironmentItem* child)
{
    //be shure child and parent where attached
    assert(environmentToNode.count(parent));
    assert(environmentToNode.count(child));
    
    {
	//as we should know visualizers for every node we can simply 
	//map to osg nodes
	environmentToNode[parent]->removeChild(environmentToNode[child]);
    }
}


void EnvireEventListener::frameNodeSet(envire::CartesianMap* map, envire::FrameNode* node)
{
    addChild(node, map);
}
 
void EnvireEventListener::frameNodeDetached(envire::CartesianMap* map, envire::FrameNode* node)
{
    removeChild(node, map);
}

void EnvireEventListener::childAdded ( envire::FrameNode* parent, envire::FrameNode* child )
{
    addChild(parent, child);
}

void EnvireEventListener::childRemoved ( envire::FrameNode* parent, envire::FrameNode* child )
{
    removeChild(parent, child);
}

void EnvireEventListener::childAdded ( envire::Layer* parent, envire::Layer* child )
{
    addChild(parent, child);
}

void EnvireEventListener::childRemoved ( envire::Layer* parent, envire::Layer* child )
{
    removeChild(parent, child);
}

EnvironmentItemVisualizer* EnvireEventListener::getVisualizerForItem(envire::EnvironmentItem* item)
{
    EnvironmentItemVisualizer *viz = 0;
    for(std::vector<EnvironmentItemVisualizer *>::reverse_iterator it = visualizers.rbegin(); it != visualizers.rend(); it++) {
	if((*it)->handlesItem(item)) {
	    viz = *it;
	    break;
	}
    }

    return viz;
}


void EnvireEventListener::itemAttached ( envire::EnvironmentItem* item )
{
    osg::Group *itemNode = 0;
    
    EnvironmentItemVisualizer *viz = getVisualizerForItem(item);
    
    //std::cout << "Item ID that was attached:"<< item->getUniqueId() << std::endl;
    //an item should never be attached two times !
    assert(environmentToNode.count(item) == 0);    
    
    //item not handled
    if(viz == 0) {
	// std::cout << "item not handled: " << item->getClassName() << std::endl;
	return;
    }
    
    itemNode = viz->getNodeForItem(item);
    
    assert(itemNode);
    
    itemNode->setDataVariance(osg::Object::DYNAMIC);        
    
    environmentToNode[item] = itemNode;
    nodeToEnvironment[itemNode] = item;    
}

void EnvireEventListener::itemDetached ( envire::EnvironmentItem* item )
{
    //we did not handle the item
    if(environmentToNode.count(item) == 0)
	return;
    
    osg::Group *group = environmentToNode[item];
    //delte item and graphical representation from map
    nodeToEnvironment.erase(group);
    
    //this group should never have a parent, as 
    //node needs to be removed from the tree
    //before it is dettached
    assert(group->getNumParents() == 0);

    //note osg node is held in an autopointer in environmentToNode
    //therefore reasing the autopointer also frees the osg::Node
    environmentToNode.erase(item);

}

void EnvireEventListener::itemModified ( envire::EnvironmentItem* item )
{
    if( !environmentToNode.count(item) )
	return;

    // TODO: it's very inefficient, to always look for the visualizer again
    // could also be stored in a map
    EnvironmentItemVisualizer *viz = getVisualizerForItem(item);
    assert(viz);
    {
	viz->updateNode(item, environmentToNode[item]);
    }
}    
    
}
