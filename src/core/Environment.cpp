#include "Core.hpp"
#include "Event.hpp"
#include "EventHandler.hpp"
#include "Serialization.hpp"

#include <algorithm>
#include <utility>
#include <stdexcept>
#include <Eigen/LU>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <iostream>

using namespace std;
using namespace envire;

const std::string EnvironmentItem::className = "envire::EnvironmentItem";

void envire::intrusive_ptr_add_ref( EnvironmentItem* item ) { item->ref_count++; }
void envire::intrusive_ptr_release( EnvironmentItem* item ) { if(!--item->ref_count) delete item; }

EnvironmentItem::EnvironmentItem(std::string const& unique_id)
    : ref_count(0), unique_id(unique_id), env(NULL)
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

void EnvironmentItem::setUniqueId(std::string const& id)
{
    if (isAttached())
        throw std::logic_error("trying to change an item's ID after it was attached to an environment");

    unique_id = id;
}

std::string EnvironmentItem::getUniqueId() const
{
    return unique_id;
}

std::string EnvironmentItem::getUniqueIdPrefix() const
{
    size_t idpos = unique_id.rfind('/');
    std::string prefix_str = "";
    if(idpos != string::npos && idpos > 0)
    {
        prefix_str = unique_id.substr(0,idpos);
    }
    return prefix_str;
}

std::string EnvironmentItem::getUniqueIdSuffix() const
{
    size_t idpos = unique_id.rfind('/');
    std::string idstr;
    if(idpos != string::npos)
    {
        idstr = unique_id.substr(idpos+1);
    }
    else
    {
        // backward compatibility
        idstr = unique_id;
    }
    return idstr;
}

Environment* EnvironmentItem::getEnvironment() const
{
    return env;
}

void EnvironmentItem::serialize(Serialization &so)
{
    so.write( "id", unique_id );
    so.write( "label", label );
}

void EnvironmentItem::unserialize(Serialization &so)
{
    so.read( "id", unique_id );

    // For backward compatibility
    if (*unique_id.begin() != '/')
        unique_id = "/" + unique_id;

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

const std::string Environment::ITEM_NOT_ATTACHED = "";

Environment::Environment() : last_id(0), envPrefix("/")
{
    // each environment has a root node
    rootNode = new FrameNode();
    rootNode->unique_id = "/0";
    last_id++;
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
	evl->handle( Event( event::FRAMENODE_TREE, event::ADD, parent, *it ) );
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
	handler->handle( Event( event::ITEM, event::ADD, it->second ) );
    }
    
    //set root node
    handler->handle( Event( event::ROOT, event::ADD, getRootNode() ) );
    
    //iterate over frame tree
    publishChilds(handler, getRootNode());    

    //publish connections between maps and Framenodes
    for(cartesianMapGraphType::iterator it = cartesianMapGraph.begin(); it != cartesianMapGraph.end(); it++) 
    {
	handler->handle( Event( event::FRAMENODE, event::ADD, it->first, it->second ) );
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
	    evl->handle( Event( event::FRAMENODE, event::REMOVE, *mapit, *it ) );
	}
	
	//and remove the leaf
	evl->handle( Event( event::FRAMENODE_TREE, event::REMOVE, parent, *it ) );
    }    
};

void Environment::removeEventHandler(EventHandler *handler) 
{
    //reverse iterate over the frame node tree and detach all children    
    detachChilds(getRootNode(), handler);
    
    //remove root node
    handler->handle( Event( event::ROOT, event::REMOVE, getRootNode() ) );
    
    //detach all environmentItems
    for(itemListType::iterator it = items.begin(); it != items.end(); it++) 
    {
	handler->handle( Event( event::ITEM, event::REMOVE, it->second ) );
    }
    
    eventHandlers.removeEventHandler( handler );
}

void Environment::setEnvironmentPrefix(std::string envPrefix)
{
    if (*envPrefix.begin() != '/')
        envPrefix = "/" + envPrefix;
    if (*envPrefix.rbegin() != '/')
        envPrefix += "/";
    this->envPrefix = envPrefix;
}

void Environment::attachItem(CartesianMap* item, FrameNode* node)
{
    attachItem(static_cast<EnvironmentItem*>(item));
    if (!item->getFrameNode())
    {
        if (node)
            item->setFrameNode(node);
        else
            item->setFrameNode(getRootNode());
    }
}

void Environment::attachItem(EnvironmentItem* item)
{
    assert( item );

    if(!item->isAttached())
    {
        if(item->unique_id == ITEM_NOT_ATTACHED)
            item->unique_id = envPrefix;

        if(*item->unique_id.begin() != '/')
            item->unique_id = envPrefix + item->unique_id;

        if(*item->unique_id.rbegin() == '/')
            item->unique_id += boost::lexical_cast<std::string>(last_id++);
    }

    // make sure item not already present
    if( items.count(item->getUniqueId()) ) {
	std::cout << "Duplicated id:" << item->getUniqueId() << std::endl;
	throw runtime_error("unique_id of item already in environment");
    }
    // add item to internal list
    items[item->getUniqueId()] = item;
   
    // set a pointer to environment object
    item->env = this;
    
    handle( Event( event::ITEM, event::ADD, item ) );
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

    handle( Event( event::ITEM, event::REMOVE, item ) );
    
    EnvironmentItem::Ptr itemPtr = items[ item->getUniqueId() ];
    items.erase( item->getUniqueId() );
    item->unique_id = ITEM_NOT_ATTACHED;
    item->env = NULL;

    return itemPtr;
}

void Environment::itemModified(EnvironmentItem* item) 
{
    handle( Event( event::ITEM, event::UPDATE, item ) );
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
    
    handle( Event( event::FRAMENODE_TREE, event::ADD, parent, child ) );
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

    handle( Event( event::LAYER_TREE, event::ADD, parent, child ) );
}

void Environment::removeChild(FrameNode* parent, FrameNode* child)
{
    if( getParent( child ) == parent )
    {
	handle( Event( event::FRAMENODE_TREE, event::REMOVE, parent, child ) );

	frameNodeTree.erase( frameNodeTree.find( child ) );
    }
}

void Environment::removeChild(Layer* parent, Layer* child) 
{
    handle( Event( event::LAYER_TREE, event::REMOVE, parent, child ) );

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

    handle( Event( event::FRAMENODE, event::ADD, map, node ) );
}

void Environment::detachFrameNode(CartesianMap* map, FrameNode* node)
{
    if( cartesianMapGraph.count(map) && cartesianMapGraph[map] == node )
    {
	handle( Event( event::FRAMENODE, event::REMOVE, map, node ) );

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
    if (from == to)
        return T( Eigen::Affine3d::Identity() );

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

FrameNode::TransformType Environment::relativeTransform(const CartesianMap* from, const CartesianMap* to)
{
    return relativeTransform( from->getFrameNode(), to->getFrameNode() );
}

TransformWithUncertainty Environment::relativeTransformWithUncertainty(const FrameNode* from, const FrameNode* to)
{
    return ::relativeTransform<TransformWithUncertainty>( from, to );
}

TransformWithUncertainty Environment::relativeTransformWithUncertainty(const CartesianMap* from, const CartesianMap* to)
{
    return relativeTransformWithUncertainty( from->getFrameNode(), to->getFrameNode() );
}

void Environment::serialize(std::string const& path)
{
    FileSerialization serialization;
    boost::filesystem::path sceneDir( path ); 
    boost::filesystem::path scene( sceneDir / serialization.STRUCTURE_FILE );

    boost::filesystem::create_directory( path );

    serialization.setSceneDir(sceneDir.string());
    serialization.writeToFile( this, scene.string() );
}

Environment* Environment::unserialize(std::string const& path)
{
    FileSerialization serialization;
    boost::filesystem::path sceneDir( path ); 
    boost::filesystem::path scene( sceneDir / serialization.STRUCTURE_FILE );

    if( !boost::filesystem::is_regular( scene ) )
    {
        std::cerr << "failed to open " << scene << std::endl;
        throw std::runtime_error("envire: could not open " + scene.string());
    }

    serialization.setSceneDir(sceneDir.string());
    return serialization.readFromFile( scene.string() );
}

