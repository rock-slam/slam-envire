#include "Core.hpp"
#include "Serialization.hpp"

#include <stdexcept>
#include <fstream>
#include <string>

#include "boost/filesystem/path.hpp"

using namespace envire;
namespace fs = boost::filesystem;

const std::string Layer::className = "envire::Layer";

Layer::Layer(std::string const& id) :
    EnvironmentItem(id), immutable(false), dirty(false)
{
}

Layer::Layer(const Layer& other) :
    EnvironmentItem( other ),
    immutable( other.immutable ),
    dirty( other.dirty )
{
    // copy the data map, and clone the holders
    for( DataMap::const_iterator it = other.data_map.begin(); it != other.data_map.end(); it++ )
	data_map.insert( std::make_pair( it->first, it->second->clone() ) );
}

Layer& Layer::operator=(const Layer& other)
{
    if( this != &other )
    {
	EnvironmentItem::operator=( other );
	immutable = other.immutable;
	dirty = other.dirty;
	removeData();
	for( DataMap::const_iterator it = other.data_map.begin(); it != other.data_map.end(); it++ )
	    data_map.insert( std::make_pair( it->first, it->second->clone() ) );
    }
    return *this;
}

void Layer::serialize(Serialization& so)
{
    EnvironmentItem::serialize(so);

    so.write( "immutable", immutable );
}

void Layer::unserialize(Serialization& so)
{
    EnvironmentItem::unserialize(so);

    so.read( "immutable", immutable );
}

Layer::~Layer()
{
    removeData();
}

void Layer::addChild( Layer* child ) 
{
    assert( env );
    env->addChild(this, child);
}

std::list<Layer*> Layer::getParents()
{
    assert( env );
    return env->getParents(this);
}

bool Layer::isImmutable() const
{
    return immutable;
}

void Layer::setImmutable()
{
    immutable = true;
}

void Layer::resetDirty() 
{
    dirty = false;
}

void Layer::setDirty() 
{
    dirty = true;
}

bool Layer::isDirty() const
{
    return dirty;
}

bool Layer::detachFromOperator()
{
    if( isGenerated() ) 
	return env->removeInput( getGenerator(), this );

    return true;
}

bool Layer::isGenerated() const 
{
    return getGenerator();
}

Operator* Layer::getGenerator() const
{
    assert( env );
    return env->getGenerator(const_cast<Layer*>(this));
}

void Layer::updateFromOperator() 
{
    if( isGenerated() && isDirty() )
        getGenerator()->updateAll();
}

const std::string Layer::getMapFileName() const 
{
    return getMapFileName(getClassName());
}

const std::string Layer::getMapFileName(const std::string& className) const 
{
    std::string uniqueId = getUniqueId();
    std::replace(uniqueId.begin(), uniqueId.end(), '/', '_');
    return className + uniqueId;
}

const std::string Layer::getMapFileName(const std::string& path, const std::string& className) const 
{
    fs::path scenePath(path); 

    std::string fileName = getMapFileName(className);

    return (scenePath / fileName).string();
}

bool Layer::hasData(const std::string& type) const
{
    return data_map.count(type);
}

void Layer::removeData(const std::string& type)
{
    if( data_map.count( type ) )
    {
	delete data_map[type];
	data_map.erase( type );
    }
}

void Layer::removeData()
{
    for( DataMap::iterator it = data_map.begin();it != data_map.end(); delete((it++)->second) );
}

const std::string CartesianMap::className = "envire::CartesianMap";

CartesianMap::CartesianMap(std::string const& id)
    : Layer(id)
{
}

void CartesianMap::setFrameNode(FrameNode* node)
{
    if(!env)
      throw std::runtime_error("Before setting the frame node add the object to the environment.");
    env->setFrameNode(this, node);
}

FrameNode* CartesianMap::getFrameNode() 
{
    return const_cast<FrameNode*>( static_cast<const CartesianMap&>( *this ).getFrameNode() );
}

const FrameNode* CartesianMap::getFrameNode() const 
{
    assert( env );
    return env->getFrameNode(const_cast<CartesianMap*>(this));
}

void CartesianMap::cloneTo(Environment& env) const
{
    CartesianMap* this_copy = clone();
    env.attachItem(this_copy);

    // Get the frame stack for +this+
    std::vector<FrameNode const*> frame_stack;
    FrameNode const* root_frame = getEnvironment()->getRootNode();
    {
        FrameNode const* frame = getFrameNode();
        while (frame != root_frame)
        {
            frame_stack.push_back(frame);
            frame = frame->getParent();
        }
    }

    // And duplicate it on the target environment
    {
        FrameNode* frame = env.getRootNode();
        while (!frame_stack.empty())
        {
            FrameNode* new_frame = frame_stack.back()->clone();
            frame_stack.pop_back();
            frame->addChild(new_frame);
            frame = new_frame;
        }
        this_copy->setFrameNode(frame);
    }
}




