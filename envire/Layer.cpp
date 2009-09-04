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

Layer::Layer(Serialization& so) : 
    EnvironmentItem(so)
{
    so.setClassName(className);

    so.read( "name", name );
    so.read( "immutable", immutable );
}

void Layer::serialize(Serialization& so)
{
    EnvironmentItem::serialize(so);
    so.setClassName(className);

    so.write( "name", name );
    so.write( "immutable", immutable );
}

Layer::~Layer()
{
}

void Layer::addChild( Layer* child ) 
{
    env->addChild(this, child);
}

Layer* Layer::getParent()
{
    return env->getParent(this);
}

std::string Layer::getName() const
{
    return name;
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

const std::string Layer::getMapFileName(const std::string& path) const 
{
    fs::path scenePath(path); 

    std::string fileName = getClassName() + "_" + boost::lexical_cast<std::string>(getUniqueId());
    if( !getName().empty() )
	fileName += getName();

    return (scenePath / fileName).string();
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

