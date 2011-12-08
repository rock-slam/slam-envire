#include "Core.hpp"
#include "Event.hpp"
#include "EventHandler.hpp"

#include <algorithm>
#include <utility>
#include <stdexcept>
#include <Eigen/LU>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

using namespace std;
using namespace envire;

const std::string EnvironmentItem::className = "envire::EnvironmentItem";

void envire::intrusive_ptr_add_ref( EnvironmentItem* item ) { item->ref_count++; }
void envire::intrusive_ptr_release( EnvironmentItem* item ) { if(!--item->ref_count) delete item; }

EnvironmentItem::EnvironmentItem()
    : ref_count(0), unique_id( Environment::ITEM_NOT_ATTACHED ), env(NULL)
{
}

EnvironmentItem::EnvironmentItem(Environment* envPtr)
   : ref_count(0), unique_id( Environment::ITEM_NOT_ATTACHED ), env(NULL)
{
    envPtr->attachItem( this );
}

EnvironmentItem::EnvironmentItem(const EnvironmentItem& item)
    : ref_count(0), unique_id( Environment::ITEM_NOT_ATTACHED ), env(NULL)
{
}

EnvironmentItem& EnvironmentItem::operator=(const EnvironmentItem& other)
{
    return *this;
}

EnvironmentItem::~EnvironmentItem()
{
    assert( !isAttached() );
}

bool EnvironmentItem::isAttached() const
{
    return env;
}

long EnvironmentItem::getUniqueId() const
{
    return unique_id;
}

Environment* EnvironmentItem::getEnvironment() const
{
    return env;
}

EnvironmentItem::EnvironmentItem(Serialization &so)
    : ref_count(0)
{
    so.setClassName(className);
    unserialize(so);
}

void EnvironmentItem::serialize(Serialization &so)
{
    so.setClassName(className);
    so.write( "id", unique_id );
    so.write( "label", label );
}

void EnvironmentItem::unserialize(Serialization &so)
{
    so.read( "id", unique_id );
    so.read( "label", label );
}
void EnvironmentItem::itemModified()
{
    if( isAttached() )
	env->itemModified(this);
}

EnvironmentItem::Ptr EnvironmentItem::detach()
{
    assert( env );
    return env->detachItem( this );
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
    itemListType::iterator it;
    while( (it = items.begin()) != items.end() )
    {
	it->second->detach();
    }
}

void Environment::publishChilds(EventHandler *evl, FrameNode *parent)
{
  
    std::list<FrameNode*> childs = getChildren(parent);
    for(std::list<FrameNode*>::iterator it = childs.begin(); it != childs.end(); it++)
    {
	evl->handle( Event( Event::FRAMENODE_TREE, Event::ADD, parent, *it ) );
	publishChilds(evl, *it);
    }
}


void Environment::handle( const Event& event )
{
    eventHandlers.handle( event );
}

void Environment::addEventHandler(EventHandler *handler) 
{
    //new listener was added, iterate over all items and 
    //attach them at the listener
    for(itemListType::iterator it = items.begin(); it != items.end(); it++) 
    {
	handler->handle( Event( Event::ITEM, Event::ADD, it->second ) );
    }
    
    //set root node
    handler->handle( Event( Event::ROOT, Event::ADD, getRootNode() ) );
    
    //iterate over frame tree
    publishChilds(handler, getRootNode());    

    //publish connections between maps and Framenodes
    for(cartesianMapGraphType::iterator it = cartesianMapGraph.begin(); it != cartesianMapGraph.end(); it++) 
    {
	handler->handle( Event( Event::FRAMENODE, Event::ADD, it->first, it->second ) );
    }
    
    eventHandlers.addEventHandler(handler);
}

void Environment::detachChilds(FrameNode *parent, EventHandler *evl)
{
    std::list<FrameNode *> childList = getChildren(parent);
    
    for(std::list<FrameNode *>::iterator it = childList.begin(); it != childList.end(); it++) {
	std::list<FrameNode *> curChildList = getChildren(*it);

	//not a leaf, detach all child recursive
	if(curChildList.size() > 0) {
	    detachChilds(*it, evl);   
	}
	
	std::list<CartesianMap*> maplist = getMaps(*it);
	
	//now it is a leaf, detach all maps attached to framenode
	for(std::list<CartesianMap*>::iterator mapit = maplist.begin(); mapit != maplist.end(); mapit++) {
	    evl->handle( Event( Event::FRAMENODE, Event::REMOVE, *mapit, *it ) );
	}
	
	//and remove the leaf
	evl->handle( Event( Event::FRAMENODE_TREE, Event::REMOVE, parent, *it ) );
    }    
};

void Environment::removeEventHandler(EventHandler *handler) 
{
    //reverse iterate over the frame node tree and detach all children    
    detachChilds(getRootNode(), handler);
    
    //remove root node
    handler->handle( Event( Event::ROOT, Event::REMOVE, getRootNode() ) );
    
    //detach all environmentItems
    for(itemListType::iterator it = items.begin(); it != items.end(); it++) 
    {
	handler->handle( Event( Event::ITEM, Event::REMOVE, it->second ) );
    }
    
    eventHandlers.removeEventHandler( handler );
}

void Environment::attachItem(EnvironmentItem* item)
{
    assert( item );

    if( item->getUniqueId() == ITEM_NOT_ATTACHED )
	item->unique_id = last_id++;

    // make sure item not already present
    if( items.count(item->getUniqueId()) ) {
	std::cout << "Duplicated id:" << item->getUniqueId() << std::endl;
	throw runtime_error("unique_id of item already in environment");
    }
    // add item to internal list
    items[item->getUniqueId()] = item;
   
    // set a pointer to environment object
    item->env = this;
    
    handle( Event( Event::ITEM, Event::ADD, item ) );
} 

template <class T>
void findMapItem(const T& map, boost::function<void (typename T::key_type, typename T::mapped_type)> func, EnvironmentItem* item, std::vector<boost::function<void ()> >& list)
{
    for(typename T::const_iterator it = map.begin(); it != map.end(); it++)
    {
	if( it->first == item || it->second == item )
	    list.push_back( boost::bind( func, it->first, it->second ) );
    }
}

EnvironmentItem::Ptr Environment::detachItem(EnvironmentItem* item, bool deep)
{
    assert( item );
    assert( items.count( item->getUniqueId() ) );

    if( deep )
    {
	// remove all associated objects first
	{
	    // for a framenode remove the children and the maps
	    FrameNode* fn = dynamic_cast<FrameNode*>( item );
	    if( fn )
	    {
		std::list<FrameNode*> children = fn->getChildren();
		for( std::list<FrameNode*>::iterator it = children.begin(); it != children.end(); it++ )
		    detachItem( *it, deep );

		std::list<CartesianMap*> maps = fn->getMaps();
		for( std::list<CartesianMap*>::iterator it = maps.begin(); it != maps.end(); it++ )
		    detachItem( *it, deep );
	    }
	}

	// TODO do the same for parent child relationsships in maps.
	// also, we might want to have the option to remove single frameNodes
	// when detaching maps, or operators when all inputs/outputs have been
	// removed
    }

    // check if there are still some references to this object left
    // effectively we need to go through all the maps, and find a reference to
    // the object 
    
    // collecting the destruction calls in some sort of list, in order to allow
    // rollback in case something can't be destructed, because of e.g.
    // subtrees.  might actually be better to never deny destruction, and have
    // some sort of cleanup method which will make sure that orphaned objects
    // will be removed.
    std::vector<boost::function<void ()> > func;

    findMapItem<frameNodeTreeType>( 
	    frameNodeTree, boost::bind( 
		static_cast<void (Environment::*)(FrameNode*,FrameNode*)>(&Environment::removeChild), 
		this, _2, _1 ), item, func );

    findMapItem<layerTreeType>( 
	    layerTree, boost::bind( 
		static_cast<void (Environment::*)(Layer*,Layer*)>(&Environment::removeChild), 
		this, _2, _1 ), item, func );

    findMapItem<operatorGraphType>( 
	    operatorGraphInput, boost::bind( 
		&Environment::removeInput, 
		this, _1, _2 ), item, func );

    findMapItem<operatorGraphType>( 
	    operatorGraphOutput, boost::bind( 
		&Environment::removeOutput, 
		this, _1, _2 ), item, func );

    findMapItem<cartesianMapGraphType>( 
	    cartesianMapGraph, boost::bind( 
		&Environment::detachFrameNode, 
		this, _1, _2 ), item, func );

    // remove the links now
    for(std::vector<boost::function<void ()> >::iterator it = func.begin(); it != func.end(); it++)
    {
	(*it)();
    }

    handle( Event( Event::ITEM, Event::REMOVE, item ) );
    
    EnvironmentItem::Ptr itemPtr = items[ item->getUniqueId() ];
    items.erase( item->getUniqueId() );
    item->unique_id = ITEM_NOT_ATTACHED;
    item->env = NULL;

    return itemPtr;
}

void Environment::itemModified(EnvironmentItem* item) 
{
    handle( Event( Event::ITEM, Event::UPDATE, item ) );
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
    
    handle( Event( Event::FRAMENODE_TREE, Event::ADD, parent, child ) );
}

void Environment::addChild(Layer* parent, Layer* child)
{
    if( !child->isAttached() )
	attachItem( child );

    /* allow multiple parents for a child
    if( getParent(child) )
    {
	removeChild( getParent(child), child );
    }
    */

    layerTree.insert(make_pair(child, parent));

    handle( Event( Event::LAYER_TREE, Event::ADD, parent, child ) );
}

void Environment::removeChild(FrameNode* parent, FrameNode* child)
{
    if( getParent( child ) == parent )
    {
	handle( Event( Event::FRAMENODE_TREE, Event::REMOVE, parent, child ) );

	frameNodeTree.erase( frameNodeTree.find( child ) );
    }
}

void Environment::removeChild(Layer* parent, Layer* child) 
{
    handle( Event( Event::LAYER_TREE, Event::REMOVE, parent, child ) );

    typedef layerTreeType::iterator iterator;
    std::pair<iterator,iterator> range = layerTree.equal_range( child );
    for( iterator it = range.first; it != range.second; it++ )
    {
	if( it->second == parent )
	{
	    layerTree.erase( it );
	    return;
	}
    }

    /*
       layerTree.erase( layerTree.find( child ) );
       */
}

FrameNode* Environment::getParent(FrameNode* node) 
{
    frameNodeTreeType::iterator it = frameNodeTree.find( node );
    if( it == frameNodeTree.end() )
	return NULL;
    else
	return it->second;
}

std::list<Layer*> Environment::getParents(Layer* layer) 
{
    std::list<Layer*> res;

    typedef layerTreeType::iterator iterator;
    std::pair<iterator,iterator> range = layerTree.equal_range( layer );
    for( iterator it = range.first; it != range.second; it++ )
	res.push_back( it->second );

    return res;

    /*
    layerTreeType::iterator it = layerTree.find( layer );
    if( it == layerTree.end() )
	return NULL;
    else
	return it->second;
	*/
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
    
    if( !map->isAttached() )
	attachItem(map);

    cartesianMapGraph[map] = node;

    handle( Event( Event::FRAMENODE, Event::ADD, map, node ) );
}

void Environment::detachFrameNode(CartesianMap* map, FrameNode* node)
{
    if( cartesianMapGraph.count(map) && cartesianMapGraph[map] == node )
    {
	handle( Event( Event::FRAMENODE, Event::REMOVE, map, node ) );

	cartesianMapGraph.erase( map );
    }
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
    if( !op->isAttached() )
	attachItem( op );

    if( !input->isAttached() )
	attachItem( input );
    
    operatorGraphInput.insert( make_pair(op, input) );

    return true;
}

bool Environment::addOutput(Operator* op, Layer* output)
{
    if( !op->isAttached() )
	attachItem( op );

    if( !output->isAttached() )
	attachItem( output );

    operatorGraphOutput.insert( make_pair(op, output) );

    return true;
}

bool Environment::removeInputs(Operator* op)
{
    return operatorGraphInput.erase( op );
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

    return true;
}

bool Environment::removeOutputs(Operator* op)
{
    return operatorGraphOutput.erase( op );
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

    return true;
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

std::list<Layer*> Environment::getLayersGeneratedFrom(Layer* input) 
{
    std::list<Layer*> result;
    for(operatorGraphType::iterator it=operatorGraphInput.begin();it != operatorGraphInput.end();++it)
    {
	if( it->second == input )
        {
            std::list<Layer*> op_output = getOutputs(it->first);
            result.splice(result.end(), op_output);
        }
    }

    return result;
}

Operator* Environment::getGenerator(Layer* output) 
{
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();++it)
    {
	if( it->second == output )
	    return it->first;
    }

    return NULL;
}

void Environment::updateOperators(){
    std::vector<envire::Operator*> ops = getItems<envire::Operator>();

    for(std::vector<envire::Operator*>::iterator it=ops.begin();it!=ops.end();it++)
    {
	(*it)->updateAll();
	
	std::list<Layer*> outs = getOutputs(*it);
	for (std::list<Layer*>::iterator out = outs.begin();out != outs.end();out++){
	    itemModified(*out);
	}
    }
}

template <class T> 
T getTransform( const FrameNode* fn ) { return fn->getTransform(); }

template <>
TransformWithUncertainty getTransform( const FrameNode* fn ) { return fn->getTransformWithUncertainty(); }

template <class T>
std::pair<T, const FrameNode*> relativeFrameNodeRoot( const FrameNode* from )
{
    T C_fg(envire::Transform(Eigen::Affine3d::Identity()));

    const FrameNode *t = from;
    while(!t->isRoot())
    {
	C_fg = getTransform<T>(t) * C_fg;
	t = t->getParent();
    }
    return make_pair( C_fg, t );
}

template <class T>
T relativeTransform(const FrameNode* from, const FrameNode* to)
{
    std::pair<T, const FrameNode*> fg = relativeFrameNodeRoot<T>(from);
    std::pair<T, const FrameNode*> tg = relativeFrameNodeRoot<T>(to);

    if( fg.second != tg.second )
	throw std::runtime_error("relativeTransform: FrameNodes don't have a common root.");

    return T( tg.first.inverse() * fg.first );
}

FrameNode::TransformType Environment::relativeTransform(const FrameNode* from, const FrameNode* to)
{
    return ::relativeTransform<FrameNode::TransformType>( from, to );
}

TransformWithUncertainty Environment::relativeTransformWithUncertainty(const FrameNode* from, const FrameNode* to)
{
    return ::relativeTransform<TransformWithUncertainty>( from, to );
}

void Environment::serialize(std::string const& path)
{
    Serialization serializer;
    serializer.serialize(this, path);
}

Environment* Environment::unserialize(std::string const& path)
{
    Serialization serializer;
    return serializer.unserialize(path);
}

