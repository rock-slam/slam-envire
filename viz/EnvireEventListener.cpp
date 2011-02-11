#include "EnvireEventListener.hpp"

using namespace envire;

namespace vizkit 
{

EnvireNode::EnvireNode( EnvironmentItem *item, EnvironmentItemVisualizer *viz )
    : item( item ), viz( viz )
{
    node.front = viz->getNodeForItem(item);
    node.front->setDataVariance(osg::Object::DYNAMIC);        

    node.back = viz->getNodeForItem(item);  
    node.back->setDataVariance(osg::Object::DYNAMIC);        
    
    children.front = new osg::Group;
    children.back = new osg::Group;
    
    update();
    this->setDataVariance(osg::Object::DYNAMIC);        
}

osg::Group* EnvireNode::getBack()
{
    return node.back;
}

osg::Group* EnvireNode::getFront()
{
    return node.front;
}

void EnvireNode::addChildNode( osg::Group* child )
{
    boost::mutex::scoped_lock lock(mu);
    
    children.back->addChild( child );
    children.dirty = true;
}

void EnvireNode::removeChildNode( osg::Group* child )
{
    boost::mutex::scoped_lock lock(mu);
    children.back->removeChild( child );
    children.dirty = true;
}

void EnvireNode::update()
{
    boost::mutex::scoped_lock lock(mu);
    viz->updateNode( item, node.back );

    node.dirty = true;
}

void EnvireNode::apply()
{
    if( mu.try_lock() )
    {
	// detach children first
	if( node.front && children.front )
	    node.front->removeChild( children.front );

	// copy references to child nodes from back to front
	// if necessary
	if( children.dirty )
	{
	    children.front->removeChildren(0, children.front->getNumChildren() );
	    for( size_t i=0; i<children.back->getNumChildren(); i++ )
		children.front->addChild( children.back->getChild( i ) );

	    children.dirty = false;
	}

	if( node.dirty )
	{
	    if( node.front )
		removeChild( node.front );

	    // swap front and back references
	    node.swap();

	    // attach new front node
	    addChild( node.front );

	    node.dirty = false;
	}

	// (re)attach the child nodes to front node
	if( node.front && children.front )
	    node.front->addChild( children.front );

	mu.unlock();
    }
}

void EnvireEventListener::apply()
{
    boost::mutex::scoped_lock lock(mu);
    typedef std::map<envire::EnvironmentItem*, osg::ref_ptr<EnvireNode> > mapType;
    for( mapType::iterator it=environmentToNode.begin(); it!=environmentToNode.end(); it++ )
    {
	// perform the swap between front and back if needed
	it->second->apply();
    }
}

EnvireEventListener::EnvireEventListener(nodeCallback add, nodeCallback remove)
    : osgAddNode(add), osgRemoveNode(remove)
{    
//	std::cout << "listener connected" << std::endl;
}

osg::Group* EnvireEventListener::getNodeForItem(envire::EnvironmentItem* item)
{
    boost::mutex::scoped_lock lock(mu);
    if(environmentToNode.count(item))
	return environmentToNode[item]->getBack();
    
    return 0;
}

osg::Group* EnvireEventListener::getParentNodeForItem(EnvironmentItem* item)
{
    boost::mutex::scoped_lock lock(mu);
    if(environmentToNode.count(item))
	return environmentToNode[item];
    
    return 0;
}

void EnvireEventListener::setRootNode(envire::FrameNode* root)
{
    boost::mutex::scoped_lock lock(mu);
    if(environmentToNode.count(root) == 0)
	itemAttached(root);
    
    osgAddNode( environmentToNode[root] );
}

void EnvireEventListener::removeRootNode(envire::FrameNode* root)
{
    boost::mutex::scoped_lock lock(mu);
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
    
    //TODO: not sure if this operation should be allowed. 
}

void EnvireEventListener::addChild(envire::EnvironmentItem* parent, envire::EnvironmentItem* child)
{
    boost::mutex::scoped_lock lock(mu);
    // if we don't have a reference to both parent and child, they are not
    // relevant for us
    if( !(environmentToNode.count(parent) && environmentToNode.count(child)) )
	return;

    environmentToNode[parent]->addChildNode(environmentToNode[child]);        
}

void EnvireEventListener::removeChild(envire::EnvironmentItem* parent, envire::EnvironmentItem* child)
{
    boost::mutex::scoped_lock lock(mu);
    // same as for addChild
    if( !(environmentToNode.count(parent) && environmentToNode.count(child)) )
	return;
    
    environmentToNode[parent]->removeChildNode(environmentToNode[child]);
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
    for(std::vector<EnvironmentItemVisualizer *>::reverse_iterator it = visualizers.rbegin(); it != visualizers.rend(); it++) 
    {
	if((*it)->handlesItem(item)) 
	    return *it;
    }

    return 0;
}


void EnvireEventListener::itemAttached ( envire::EnvironmentItem* item )
{
    //an item should never be attached twice !
    assert(environmentToNode.count(item) == 0);    

    EnvironmentItemVisualizer *viz = getVisualizerForItem(item);
    //item not handled
    if(viz == 0) {
	// std::cout << "item not handled: " << item->getClassName() << std::endl;
	return;
    }
    
    EnvireNode* itemNode = new EnvireNode( item, viz ); 
    
    boost::mutex::scoped_lock lock(mu);
    environmentToNode[item] = itemNode;
}

void EnvireEventListener::itemDetached ( envire::EnvironmentItem* item )
{
    boost::mutex::scoped_lock lock(mu);
    // we did not handle the item
    if(environmentToNode.count(item) == 0)
	return;
    
    environmentToNode.erase(item);
}

void EnvireEventListener::itemModified ( envire::EnvironmentItem* item )
{
    boost::mutex::scoped_lock lock(mu);
    if( !environmentToNode.count(item) )
	return;

    environmentToNode[item]->update();
}    
    
}
