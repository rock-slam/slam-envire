#include "FrameNodeVisualization.hpp"
#include <envire/Core.hpp>
#include <osg/PositionAttitudeTransform>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>

namespace envire {

osg::Group* FrameNodeVisualization::getNodeForItem(envire::EnvironmentItem* item) const
{
    osg::PositionAttitudeTransform *transform = new osg::PositionAttitudeTransform;

    updateNode(item, transform);
    
    return transform;
}

bool FrameNodeVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    return dynamic_cast<envire::FrameNode *>(item);
}

void FrameNodeVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    envire::FrameNode *fn = dynamic_cast<envire::FrameNode *>(item);
    osg::PositionAttitudeTransform *transform = dynamic_cast<osg::PositionAttitudeTransform *>(group);
    
    assert(fn && transform);
    
    const Eigen::Quaterniond quat(fn->getTransform().rotation());
    const Eigen::Vector3d translation = fn->getTransform().translation();
    
    transform->setPosition(osg::Vec3f( translation.x(), translation.y(), translation.z()));
    transform->setAttitude(osg::Quat( quat.x(), quat.y(), quat.z(), quat.w()));
}

void FrameNodeVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{    
    osg::PositionAttitudeTransform *transform = dynamic_cast<osg::PositionAttitudeTransform *>(group);    
    assert(transform);
    
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3d(0, 0, 0), 0.3);
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp.get());
    spd->setColor(osg::Vec4(1.0, 0, 0, 1.0));
    osg::ref_ptr<osg::Geode> gp = new osg::Geode();
    gp->addDrawable(spd.get());
    
    gp->setName("FrameNodeHighlighter");
    transform->addChild(gp);
}

void FrameNodeVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::PositionAttitudeTransform *transform = dynamic_cast<osg::PositionAttitudeTransform *>(group);
    assert(transform);

    for(unsigned int i = 0; i < transform->getNumChildren();i++) {
	if(group->getChild(i)->getName() == std::string("FrameNodeHighlighter")) {
	    group->removeChild(i);
	    break;
	}
    }
}

/*
* Changes the NodeMask of the given osg Node, so it gets invisible.
*/
void FrameNodeVisualization::hideNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    group->setNodeMask(INVISIBLE_MASK);
}

/*
* Changes the NodeMask of the given osg Node, so it gets visible.
*/
void FrameNodeVisualization::unHideNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    group->setNodeMask(VISIBLE_MASK);
}

}
