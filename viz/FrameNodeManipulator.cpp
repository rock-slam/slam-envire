#include "FrameNodeManipulator.hpp"
#include <osg/Version>

using namespace envire;

#if OSG_MIN_VERSION_REQUIRED(3,0,0) 
class FrameNodeUpdate : public osgManipulator::DraggerCallback
{
    osg::ref_ptr<osg::MatrixTransform> tf;
    envire::FrameNode::Ptr fn;

public:
    FrameNodeUpdate( osg::MatrixTransform* tf, envire::FrameNode* fn )
	: tf( tf ), fn( fn )
    {
    }

    bool receive( const osgManipulator::MotionCommand& cmd )
    {
	// read matrix from transform osgnode and write it to the
	// transform of the framenode
	fn->setTransform( 
		Eigen::Affine3d( 
		    Eigen::Map<const Eigen::Matrix<osg::Matrix::value_type, 4, 4> >( 
			tf->getMatrix().ptr() ) ) );
	return true;
    }
};

// copied from the osgmanipulator example
// The DraggerContainer node is used to fix the dragger's size on the screen
class DraggerContainer : public osg::Group
{
public:
    DraggerContainer() : _draggerSize(240.0f), _active(true) {}
    
    DraggerContainer( const DraggerContainer& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY )
    :   osg::Group(copy, copyop),
        _dragger(copy._dragger), _draggerSize(copy._draggerSize), _active(copy._active)
    {}
    
    META_Node( osgManipulator, DraggerContainer );
    
    void setDragger( osgManipulator::Dragger* dragger )
    {
        _dragger = dragger;
        if ( !containsNode(dragger) ) addChild( dragger );
    }
    
    osgManipulator::Dragger* getDragger() { return _dragger.get(); }
    const osgManipulator::Dragger* getDragger() const { return _dragger.get(); }
    
    void setDraggerSize( float size ) { _draggerSize = size; }
    float getDraggerSize() const { return _draggerSize; }
    
    void setActive( bool b ) { _active = b; }
    bool getActive() const { return _active; }
    
    void traverse( osg::NodeVisitor& nv )
    {
        if ( _dragger.valid() )
        {
            if ( _active && nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
            {
                osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
                
                float pixelSize = cv->pixelSize(_dragger->getBound().center(), 0.48f);
                if ( pixelSize!=_draggerSize )
                {
                    float pixelScale = pixelSize>0.0f ? _draggerSize/pixelSize : 1.0f;
                    osg::Vec3d scaleFactor(pixelScale, pixelScale, pixelScale);
                    
                    osg::Vec3 trans = _dragger->getMatrix().getTrans();
                    _dragger->setMatrix( osg::Matrix::scale(scaleFactor) * osg::Matrix::translate(trans) );
                }
            }
        }
        osg::Group::traverse(nv);
    }
    
protected:
    osg::ref_ptr<osgManipulator::Dragger> _dragger;
    float _draggerSize;
    bool _active;
};
#endif

FrameNodeManipulator::FrameNodeManipulator(envire::EnvironmentItem* item, osg::Group* pNode )
{
#if OSG_MIN_VERSION_REQUIRED(3,0,0) 
    parentNode = pNode;
    fr = dynamic_cast<envire::FrameNode *>(item);

    assert(fr && parentNode);

    // add new transform node as selection for dragger
    selection = new osg::MatrixTransform;
    parentNode->addChild(selection);

    // the draggercallback is responsible for writing the result
    // from the selection matrix node to the framenode in envire
    draggerCallback = new FrameNodeUpdate( selection, fr );

    // set up two different draggers
    osgManipulator::TranslateAxisDragger *translateDragger = new osgManipulator::TranslateAxisDragger();
    translateDragger->setupDefaultGeometry();
    translateDragger->addDraggerCallback( draggerCallback );
    translateDragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_SHIFT);
    translateDragger->setHandleEvents(true);

    osgManipulator::TrackballDragger *rotateDragger = new osgManipulator::TrackballDragger();
    rotateDragger->setupDefaultGeometry();
    rotateDragger->addDraggerCallback( draggerCallback );
    rotateDragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_SHIFT);
    rotateDragger->setHandleEvents(true);

    // create a parent child relationship between the draggers
    translateDragger->addDragger( rotateDragger );
    translateDragger->addChild( rotateDragger );

    DraggerContainer* draggerContainer = new DraggerContainer;
    draggerContainer->setDragger( translateDragger );
    dragger = draggerContainer;

    // and add them to the parent node, so they are in the osg
    // tree
    parentNode->addChild( draggerContainer );

    // and make them update the selection node
    translateDragger->addTransformUpdating( selection );
    rotateDragger->addTransformUpdating( selection );
#endif
}

FrameNodeManipulator::~FrameNodeManipulator()
{
#if OSG_MIN_VERSION_REQUIRED(3,0,0) 
    parentNode->removeChild(dragger);
    parentNode->removeChild(selection);
#endif
}
