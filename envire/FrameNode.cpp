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

Frame const& FrameNode::getTransform() const 
{
    return const_cast<Frame&>(static_cast<const FrameNode&>(*this).getTransform());
}

Frame& FrameNode::getTransform()
{
    if( isRoot() )
        throw std::runtime_error("Called getTransform() on root FrameNode.");
    return frame;
}

void FrameNode::setTransform(Frame const& transform)
{
    frame = transform;
}

