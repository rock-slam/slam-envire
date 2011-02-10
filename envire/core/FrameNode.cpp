#include "Core.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

FrameNode::FrameNode()
    : frame( Transform( Eigen::Transform3d::Identity()) )
{
}

FrameNode::FrameNode(const TransformType& t)
    : frame( t )
{
}

FrameNode::FrameNode(Serialization &so)
    : EnvironmentItem( so )
{
    so.setClassName(className);
    Eigen::Transform3d t;
    so.read("transform", t );
    frame.setTransform( t );
}

void FrameNode::serialize(Serialization &so)
{
    EnvironmentItem::serialize( so );

    so.setClassName(className);
    so.write("transform", frame.getTransform() );
}

bool FrameNode::isRoot() const
{
    return !getParent();
}

void FrameNode::addChild( FrameNode *child )
{
    env->addChild( this, child );
}

const FrameNode* FrameNode::getParent() const
{
    return env->getParent(const_cast<FrameNode*>(this));
}

FrameNode* FrameNode::getParent()
{
    return const_cast<FrameNode*>(static_cast<const FrameNode&>(*this).getParent());
}

FrameNode::TransformType const& FrameNode::getTransform() const 
{
    return frame.getTransform();
}

void FrameNode::setTransform(TransformType const& transform)
{
    frame = TransformWithUncertainty( transform );

    if(env) {
	env->itemModified(this);
    }
}

TransformWithUncertainty const& FrameNode::getTransformWithUncertainty() const
{
    return frame;
}

void FrameNode::setTransform(TransformWithUncertainty const& transform)
{
    frame = transform;

    if(env) {
	env->itemModified(this);
    }
}

FrameNode::TransformType FrameNode::relativeTransform( const FrameNode* to ) const
{
    return env->relativeTransform( this, to );
}
