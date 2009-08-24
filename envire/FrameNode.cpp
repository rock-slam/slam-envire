#include "Core.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

FrameNode::FrameNode()
    : frame( Eigen::Matrix4f::Identity() )
{
}

FrameNode::FrameNode(Serialization &so)
    : EnvironmentItem( so )
{
    so.setClassName("envire::FrameNode");
    so.read("transform", frame);
}

void FrameNode::serialize(Serialization &so)
{
    so.setClassName("envire::FrameNode");
    so.write("transform", frame);

    EnvironmentItem::serialize( so );
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

FrameNode::TransformType const& FrameNode::getTransform() const 
{
    return const_cast<TransformType&>(static_cast<const FrameNode&>(*this).getTransform());
}

FrameNode::TransformType& FrameNode::getTransform()
{
    return frame;
}

void FrameNode::setTransform(TransformType const& transform)
{
    frame = transform;
}

