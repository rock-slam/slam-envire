#include "FrameNodeManipulator.hpp"
#include <osg/Version>
#include <envire/core/EventHandler.hpp>

using namespace envire;

#if OPENSCENEGRAPH_MAJOR_VERSION >= 3 
class FrameNodeUpdate : public osgManipulator::DraggerCallback, public envire::EventHandler
{
    osg::ref_ptr<osg::MatrixTransform> tf;
    envire::FrameNode::Ptr fn;
    osg::ref_ptr<osgManipulator::Dragger> translate, rotate;

public:
    FrameNodeUpdate( osg::MatrixTransform* tf, envire::FrameNode* fn, osgManipulator::Dragger* translate, osgManipulator::Dragger* rotate )
	: tf( tf ), fn( fn ), translate( translate ), rotate( rotate )
    {
	assert( fn && tf && translate && rotate );

	// set initial transform from envire
	updateOSG();
	
	// register callbacks
	fn->getEnvironment()->addEventHandler( this );
	translate->addDraggerCallback( this );
	rotate->addDraggerCallback( this );
    }

    ~FrameNodeUpdate()
    {
	// remove callbacks
	fn->getEnvironment()->removeEventHandler( this );
	translate->removeDraggerCallback( this );
	rotate->removeDraggerCallback( this );
    }

    /** 
     * callback for the osg side 
     *
     * CAUTION we are actually going over thread boundaries here.
     * this implementation may lead to invalid matrices.
     */
    bool receive( const osgManipulator::MotionCommand& cmd )
    {
	updateEnvire();
	return true;
    }

    /** 
     * callback for the envire side
     *
     * CAUTION we are actually going over thread boundaries here.
     * this implementation may lead to invalid matrices.
     */
    void handle( const envire::Event& message )
    {
	if( message.a == fn.get() && message.operation == envire::event::UPDATE )
	    updateOSG();
    }

    void updateEnvire()
    {
	// read matrix from transform osgnode and write it to the
	// transform of the framenode
	fn->setTransform( 
		Eigen::Affine3d( 
		    Eigen::Map<const Eigen::Matrix<osg::Matrix::value_type, 4, 4> >( 
			tf->getMatrix().ptr() ) ) );
    }

    void updateOSG()
    {
	Eigen::Affine3d t = fn->getTransform();
	// update the selection first
	tf->setMatrix( 
		osg::Matrixd( t.matrix().data() ) );

	// and then the draggers
	rotate->setMatrix( osg::Matrix::scale(rotate->getMatrix().getScale()) * 
		osg::Matrixd( Eigen::Affine3d( t.linear() ).matrix().data() ) );
	translate->setMatrix( osg::Matrix::scale(translate->getMatrix().getScale()) * 
		osg::Matrixd( Eigen::Affine3d( Eigen::Translation3d(t.translation()) ).matrix().data() ) );
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
#if OPENSCENEGRAPH_MAJOR_VERSION >= 3 
    parentNode = pNode;
    fr = dynamic_cast<envire::FrameNode *>(item);

    assert(fr && parentNode);

    // add new transform node as selection for dragger
    selection = new osg::MatrixTransform;
    parentNode->addChild(selection);


    // set up two different draggers
    osgManipulator::TranslateAxisDragger *translateDragger = new osgManipulator::TranslateAxisDragger();
    translateDragger->setupDefaultGeometry();
    translateDragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_SHIFT);
    translateDragger->setActivationKeyEvent(osgGA::GUIEventAdapter::KEY_M);
    translateDragger->setHandleEvents(true);

    osgManipulator::TrackballDragger *rotateDragger = new osgManipulator::TrackballDragger();
    rotateDragger->setupDefaultGeometry();
    rotateDragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_SHIFT);
    rotateDragger->setActivationKeyEvent(osgGA::GUIEventAdapter::KEY_M);
    // change scaling of the rotation dragger
    rotateDragger->setMatrix( osg::Matrix::scale(rotateDragger->getMatrix().getScale() * 0.6) * osg::Matrix::translate(rotateDragger->getMatrix().getTrans()) );
    rotateDragger->setHandleEvents(true);

    // create a parent child relationship between the draggers
    translateDragger->addDragger( rotateDragger );
    translateDragger->addChild( rotateDragger );

    DraggerContainer* draggerContainer = new DraggerContainer;
    draggerContainer->setDragger( translateDragger );
    dragger = draggerContainer;

    // and add them to the parent node, so they are in the osg
    // tree
    parentNode->addChild( dragger );

    // and make them update the selection node
    translateDragger->addTransformUpdating( selection );
    rotateDragger->addTransformUpdating( selection );

    // the draggercallback is responsible for writing the result
    // from the selection matrix node to the framenode in envire
    draggerCallback = new FrameNodeUpdate( selection, fr, translateDragger, rotateDragger );

#endif
}

FrameNodeManipulator::~FrameNodeManipulator()
{
#if OPENSCENEGRAPH_MAJOR_VERSION >= 3 
    parentNode->removeChild(dragger);
    parentNode->removeChild(selection);
#endif
}
