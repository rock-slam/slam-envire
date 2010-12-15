#ifndef FRAMENODESELECTION_H
#define FRAMENODESELECTION_H

#include <osgManipulator/Selection>
#include <envire/Core.hpp>

namespace vizkit {

    
/**
* This class get's MotionCommand from an osg::Dragger
* and applies them to the given FrameNode.
*/
class FrameNodeSelection : public osgManipulator::Selection
{
    public:
    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FrameNodeSelection(envire::FrameNode *fn);
	virtual bool receive(const osgManipulator::MotionCommand& );
	virtual bool receive(const osgManipulator::TranslateInLineCommand& command);
	virtual bool receive(const osgManipulator::TranslateInPlaneCommand& command);
	virtual bool receive(const osgManipulator::Scale1DCommand& command);
	virtual bool receive(const osgManipulator::Scale2DCommand& command);
	virtual bool receive(const osgManipulator::ScaleUniformCommand& command);
	virtual bool receive(const osgManipulator::Rotate3DCommand& command);    
	
    private:
	envire::FrameNode *frameNode;
	envire::FrameNode::TransformType startPoint;
};
}

#endif // ENVIRESELECTION_H
