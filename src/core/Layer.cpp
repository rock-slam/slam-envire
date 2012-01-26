#include "Core.hpp"

#include <stdexcept>
#include <fstream>
#include <string>

#include "boost/filesystem/path.hpp"

using namespace envire;
namespace fs = boost::filesystem;

const std::string Layer::className = "envire::Layer";

Layer::Layer() :
    immutable(false)
{
}

Layer::Layer(Serialization& so)
{
    unserialize(so);
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
    for( std::map<std::string, HolderBase*>::iterator it = data_map.begin();it != data_map.end(); delete((it++)->second) );
}

void Layer::addChild( Layer* child ) 
{
    env->addChild(this, child);
}

std::list<Layer*> Layer::getParents()
{
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
    return className + "_" + boost::lexical_cast<std::string>(getUniqueId());
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
    data_map.erase( type );
}

const std::string CartesianMap::className = "envire::CartesianMap";

CartesianMap::CartesianMap() 
{
}

CartesianMap::CartesianMap(Serialization& so) :
    Layer(so)
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




