#include "FrameNodeVisualization.hpp"
#include <envire/Core.hpp>
#include <osg/PositionAttitudeTransform>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <vizkit3d/Uncertainty.hpp>

namespace envire {

FrameNodeVisualization::FrameNodeVisualization() : showUncertainty(false)
{
}

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

void setTransform( osg::PositionAttitudeTransform *transform, const Eigen::Affine3d& trans )
{
    const Eigen::Quaterniond quat(trans.linear());
    const Eigen::Vector3d translation = trans.translation();
    
    transform->setPosition(osg::Vec3f( translation.x(), translation.y(), translation.z()));
    transform->setAttitude(osg::Quat( quat.x(), quat.y(), quat.z(), quat.w()));
}

void FrameNodeVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    envire::FrameNode *fn = dynamic_cast<envire::FrameNode *>(item);
    osg::PositionAttitudeTransform *transform = dynamic_cast<osg::PositionAttitudeTransform *>(group);
    
    assert(fn && transform);
    setTransform( transform, fn->getTransform() );
    // slightly awkward way of adding the uncertainty,
    // since it should actually be added to the parent
    // frame. adds a group with the inverse transform to 
    // emulate the same effect
    if( fn->getTransformWithUncertainty().hasUncertainty() )
    {
	const std::string name = "FrameNodeUncertaintyGroup";
	osg::PositionAttitudeTransform *ug = NULL;
	for( size_t i=0; i<group->getNumChildren(); ++i )
	{
	    if( group->getChild(i)->getName() == name )
	       ug = dynamic_cast<osg::PositionAttitudeTransform*>(group->getChild(i));
	}
	if( !ug )
	{
	    // use the inverse transform
	    ug = new osg::PositionAttitudeTransform();
	    ug->setName( name );
	    group->addChild( ug );
	}
	setTransform( ug, fn->getTransform().inverse() );

	ug->removeChildren(0, ug->getNumChildren());

        if(showUncertainty)
        {
            TransformWithUncertainty tf = fn->getTransformWithUncertainty();
            vizkit3d::Uncertainty *ellipse = new vizkit3d::Uncertainty();
            ellipse->setMean( Eigen::Vector3d(tf.getTransform().translation()) );
            ellipse->setCovariance( Eigen::Matrix3d( tf.getCovariance().bottomRightCorner<3,3>()) );
            ug->addChild( ellipse );
        }
    }
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
