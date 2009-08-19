#include "Core.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

FrameNode::FrameNode()
{
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

