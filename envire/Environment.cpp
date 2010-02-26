#include "Core.hpp"

#include <algorithm>
#include <utility>
#include <stdexcept>
#include <Eigen/LU>

using namespace std;
using namespace envire;

const std::string EnvironmentItem::className = "envire::EnvironmentItem";

EnvironmentItem::EnvironmentItem()
    : env(NULL), unique_id( Environment::ITEM_NOT_ATTACHED )
{
}

EnvironmentItem::EnvironmentItem(Environment* envPtr)
   : unique_id( Environment::ITEM_NOT_ATTACHED ) 
{
    envPtr->attachItem( this );
}

EnvironmentItem::~EnvironmentItem()
{
}

bool EnvironmentItem::isAttached() const
{
    return env;
}

long EnvironmentItem::getUniqueId() const
{
    return unique_id;
}

Environment* EnvironmentItem::getEnvironment() 
{
    return env;
}

EnvironmentItem::EnvironmentItem(Serialization &so)
{
    so.setClassName(className);
    so.read( "id", unique_id );
}

void EnvironmentItem::serialize(Serialization &so)
{
    so.setClassName(className);
    so.write( "id", unique_id );
}


Environment::Environment() :
    last_id(0)
{
    // each environment has a root node
    rootNode = new FrameNode();
    // also put it in the same managed process
    attachItem( rootNode );
}

Environment::~Environment() 
{
    // perform a delete on all the owned objects
    for(itemListType::iterator it=items.begin();it!=items.end();++it)
    {
	delete (*it).second;
    }
}

void Environment::publishChilds(EventListener *evl, FrameNode *parent)
{
  
    std::list<FrameNode*> childs = getChildren(parent);
    for(std::list<FrameNode*>::iterator it = childs.begin(); it != childs.end(); it++)
    {
	evl->childAdded(parent, *it);
	publishChilds(evl, *it);
    }
}

void Environment::addEventListener(EventListener *evl) 
{
    //new listener was added, iterate over all items and 
    //attach them at the listener
    for(itemListType::iterator it = items.begin(); it != items.end(); it++) 
    {
	evl->itemAttached((*it).second );
    }
    
    //iterate over frame tree
    publishChilds(evl, getRootNode());    

    //publish connections between maps and Framenodes
    for(cartesianMapGraphType::iterator it = cartesianMapGraph.begin(); it != cartesianMapGraph.end(); it++) 
    {
	evl->frameNodeSet(it->first, it->second);
    }
    
    eventListeners.push_back(evl);
}

void Environment::removeEventListener(EventListener *evl) 
{
    eventListenerType::iterator it = std::find(eventListeners.begin(), eventListeners.end(), evl);
    if(it != eventListeners.end()) {
	eventListeners.erase(it);
    }
}

void Environment::attachItem(EnvironmentItem* item)
{
    if( item->getUniqueId() == ITEM_NOT_ATTACHED )
	item->unique_id = last_id++;

    // make sure item not already present
    if( items.count(item->getUniqueId()) ) {
	std::cout << "Duplicated id:" << item->getUniqueId() <<std::endl;
	throw runtime_error("unique_id of item already in environment");
    }
    // add item to internal list
    items[item->getUniqueId()] = item;

    // set a pointer to environment object
    item->env = this;
    
    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->itemAttached(item);
    }
} 

void Environment::detachItem(EnvironmentItem* item)
{
    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->itemDetached(item);
    }
    
    items.erase( item->getUniqueId() );
    item->unique_id = ITEM_NOT_ATTACHED;
}

void Environment::itemModified(EnvironmentItem* item) {
    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->itemModified(item);
    }    
}

void Environment::addChild(FrameNode* parent, FrameNode* child)
{
    if( !child->isAttached() )
	attachItem( child );

    if( getParent(child) )
    {
	removeChild( getParent(child), child );
    }

    frameNodeTree.insert(make_pair(child, parent));
    
    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->childAdded(parent, child);
    }
}

void Environment::addChild(Layer* parent, Layer* child)
{
    if( !child->isAttached() )
	attachItem( child );

    if( getParent(child) )
    {
	removeChild( getParent(child), child );
    }

    layerTree.insert(make_pair(child, parent));

    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->childAdded(parent, child);
    }
}

void Environment::removeChild(FrameNode* parent, FrameNode* child)
{
    if( getParent( child ) == parent )
    {
	for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
	{
	    (*it)->childRemoved(parent, child);
	}

	frameNodeTree.erase( frameNodeTree.find( child ) );
    }
}

void Environment::removeChild(Layer* parent, Layer* child) 
{
    if( getParent( child ) == parent )
    {
	for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
	{
	    (*it)->childRemoved(parent, child);
	}

	layerTree.erase( layerTree.find( child ) );
    }
}

FrameNode* Environment::getParent(FrameNode* node) 
{
    frameNodeTreeType::iterator it = frameNodeTree.find( node );
    if( it == frameNodeTree.end() )
	return NULL;
    else
	return it->second;
}

Layer* Environment::getParent(Layer* layer) 
{
    layerTreeType::iterator it = layerTree.find( layer );
    if( it == layerTree.end() )
	return NULL;
    else
	return it->second;
}

FrameNode* Environment::getRootNode() 
{
    return rootNode;
}

std::list<FrameNode*> Environment::getChildren(FrameNode* parent)
{
    std::list<FrameNode*> children;
    for(frameNodeTreeType::iterator it=frameNodeTree.begin();it != frameNodeTree.end(); ++it )
    {
	if( it->second == parent )
	    children.push_back( it->first );
    }

    return children;
}

std::list<Layer*> Environment::getChildren(Layer* parent) 
{
    std::list<Layer*> children;
    for(layerTreeType::iterator it=layerTree.begin();it != layerTree.end(); ++it )
    {
	if( it->second == parent )
	    children.push_back( it->first );
    }

    return children;
}

void Environment::setFrameNode(CartesianMap* map, FrameNode* node)
{
    if( !node->isAttached() )
	attachItem( node );

    for(eventListenerType::iterator it = eventListeners.begin(); it != eventListeners.end(); it++) 
    {
	(*it)->frameNodeSet(map, node);
    }

    cartesianMapGraph.insert( make_pair( map, node ) );
}

FrameNode* Environment::getFrameNode(CartesianMap* map)
{
    return cartesianMapGraph[map];
}

std::list<CartesianMap*> Environment::getMaps(FrameNode* node) 
{
    std::list<CartesianMap*> maps;
    for(cartesianMapGraphType::iterator it=cartesianMapGraph.begin();it != cartesianMapGraph.end(); ++it )
    {
	if( it->second == node )
	    maps.push_back( it->first );
    }
    return maps;
}

bool Environment::addInput(Operator* op, Layer* input)
{
    if( !input->isAttached() )
	attachItem( input );
    
    operatorGraphInput.insert( make_pair(op, input) );
}

bool Environment::addOutput(Operator* op, Layer* output)
{
    if( !output->isAttached() )
	attachItem( output );

    operatorGraphOutput.insert( make_pair(op, output) );
}

bool Environment::removeInput(Operator* op, Layer* input)
{
    for(operatorGraphType::iterator it=operatorGraphInput.begin();it != operatorGraphInput.end();)
    {
	if( it->first == op && it->second == input )
	    operatorGraphInput.erase( it++ );
	else
	    ++it;
    }
}

bool Environment::removeOutput(Operator* op, Layer* output)
{
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();)
    {
	if( it->first == op && it->second == output )
	    operatorGraphOutput.erase( it++ );
	else
	    ++it;
    }
}

std::list<Layer*> Environment::getInputs(Operator* op)
{
    std::list<Layer*> inputs;
    for(operatorGraphType::iterator it=operatorGraphInput.begin();it != operatorGraphInput.end();++it)
    {
	if( it->first == op )
	    inputs.push_back( it->second );
    }
    return inputs;
}

std::list<Layer*> Environment::getOutputs(Operator* op) 
{
    std::list<Layer*> outputs;
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();++it)
    {
	if( it->first == op )
	    outputs.push_back( it->second );
    }
    return outputs;
}

Operator* Environment::getGenerator(Layer* output) 
{
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();++it)
    {
	if( it->second == output )
	    return it->first;
    }
}


FrameNode::TransformType Environment::relativeTransform(const FrameNode* from, const FrameNode* to)
{
    // the simplest but not most efficient way is to go through the root node
    // a better way would be to look for the closest common ancestor...

    FrameNode::TransformType C_fg(Eigen::Matrix4d::Identity()), C_tg(Eigen::Matrix4d::Identity());
    const FrameNode* t = from;
    while(!t->isRoot())
    {
	C_fg = t->getTransform() * C_fg;
	t = t->getParent();

	if( !t )
	{
	    throw std::runtime_error("FrameNode is not in main FrameTree");
	}
    }

    t = to;
    while(!t->isRoot())
    {
	C_tg = t->getTransform() * C_tg;
	t = t->getParent();

	if( !t )
	{
	    throw std::runtime_error("FrameNode is not in main FrameTree");
	}
    }

    return FrameNode::TransformType( C_tg.inverse() * C_fg );
}

bool Environment::loadSceneFile( const std::string& fileName )
{
    // TODO: define a better format for a scene file, which supports
    // hierarchical scenes
#if 0
    std::ifstream ifile(fileName.c_str());
    if( ifile.fail() )
    {
        throw std::runtime_error("Could not open file '" + fileName + "'.");
    }
    else try
    {
        std::string line;
        while( !ifile.eof() ) {
            getline( ifile, line );
            std::istringstream iline( line );

            if( line.length() > 0 ) {
                std::string filePath;
                Eigen::Transform3f t;

                iline >> filePath;
                for(int j=0;j<16;j++) {
                    iline >> t.data()[j];
                }

                // TODO this is really a bit messy as it assumes that the
                // createFromScanFile creates a new FrameNode, which it might not.
                // since this code will change anyway, this has been left out so
                // far.
                LaserScan_Ptr scan = LaserScan::createFromScanFile( filePath, getFrameNode() );
                Eigen::Quaternionf q = Eigen::Quaternionf(t.matrix().corner<3,3>(Eigen::TopLeft));
                Eigen::Vector3f v = t.translation();

                if( scan->getFrameNode()->isRoot() )
                {
                    FrameNode_Ptr node = FrameNode_Ptr( new envire::FrameNode() );
                    node->setParent( scan->getFrameNode() );
                    scan->setFrameNode( node );
                }

                scan->getFrameNode()->getTransform().getRotation() = q;
                scan->getFrameNode()->getTransform().getTranslation() = v;
            }
        }

        ifile.close();
    } 
    catch( ... )
    {
        ifile.close();
        throw;
    }
#endif
}

