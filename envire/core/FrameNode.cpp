#include "Core.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

const std::string FrameNode::className = "envire::FrameNode";

FrameNode::FrameNode()
    : frame( Eigen::Matrix4d::Identity() )
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
    so.read("transform", frame);
}

void FrameNode::serialize(Serialization &so)
{
    EnvironmentItem::serialize( so );

    so.setClassName(className);
    so.write("transform", frame);
}

bool FrameNode::isRoot() const
{
    return !getParent();
}

const FrameNode* FrameNode::getParent() const
{
    return env->getParent(const_cast<FrameNode*>(this));
}

FrameNode* FrameNode::getParent()
{
    return const_cast<FrameNode*>(static_cast<const FrameNode&>(*this).getParent());
}

FrameNode::TransformType FrameNode::getTransform() const 
{
    read_lock( mutex );
    return frame;
}

void FrameNode::setTransform(TransformType const& transform)
{
    write_lock( mutex );
    frame = transform;
    if(env) {
	env->itemModified(this);
    }
}

