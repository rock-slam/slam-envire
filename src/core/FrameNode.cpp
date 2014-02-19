#include "Core.hpp"
#include "Serialization.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

ENVIRONMENT_ITEM_DEF( FrameNode )

FrameNode::FrameNode()
    : EnvironmentItem(Environment::ITEM_NOT_ATTACHED)
    , frame( Transform( Eigen::Affine3d::Identity()) )
{
}

FrameNode::FrameNode(const TransformWithUncertainty& t)
    : EnvironmentItem(Environment::ITEM_NOT_ATTACHED)
    , frame( t )
{
}

FrameNode::FrameNode(const Transform& t)
    : EnvironmentItem(Environment::ITEM_NOT_ATTACHED)
    , frame( t )
{
}

void FrameNode::serialize(Serialization &so)
{
    EnvironmentItem::serialize( so );

    so.write("transform", frame.getTransform() );
    if(frame.hasUncertainty())
        so.write("covariance", frame.getCovariance() );
}

void FrameNode::unserialize(Serialization &so)
{
    EnvironmentItem::unserialize(so);
    
    Eigen::Affine3d t;
    so.read("transform", t );
    frame.setTransform( t );
    if( so.hasKey( "covariance" ) )
    {
        TransformWithUncertainty::Covariance cov;
        so.read("covariance", cov );
        frame.setCovariance( cov );
    }
}

bool FrameNode::isRoot() const
{
    return !getParent();
}

void FrameNode::addChild( FrameNode *child )
{
    env->addChild( this, child );
}

const FrameNode* FrameNode::getRoot() const
{
    const FrameNode* frame = this;
    while (!frame->isRoot())
        frame = frame->getParent();
    return frame;
}

FrameNode* FrameNode::getRoot()
{
    return const_cast<FrameNode*>(static_cast<const FrameNode&>(*this).getRoot());
}

const FrameNode* FrameNode::getParent() const
{
    return env->getParent(const_cast<FrameNode*>(this));
}

FrameNode* FrameNode::getParent()
{
    return const_cast<FrameNode*>(static_cast<const FrameNode&>(*this).getParent());
}

Transform const& FrameNode::getTransform() const 
{
    return frame.getTransform();
}

void FrameNode::setTransform(Transform const& transform)
{
    frame = TransformWithUncertainty( transform );

    if(isAttached()) {
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

    if(isAttached()) {
	env->itemModified(this);
    }
}

Transform FrameNode::relativeTransform( const FrameNode* to ) const
{
    return env->relativeTransform( this, to );
}

std::list<CartesianMap*> FrameNode::getMaps()
{
    return env->getMaps( this );
}

std::list<FrameNode*> FrameNode::getChildren()
{
    return env->getChildren( this );
}
