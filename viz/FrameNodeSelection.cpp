#include "FrameNodeSelection.hpp"
#include <osg/Version>
#include <Eigen/LU>

namespace vizkit {

FrameNodeSelection::FrameNodeSelection(envire::FrameNode *fn) :frameNode(fn)
{

}

bool FrameNodeSelection::receive(const osgManipulator::MotionCommand& command)
{
#if OPENSCENEGRAPH_MAJOR_VERSION < 3
    switch (command.getStage())
    {
        case osgManipulator::MotionCommand::START:
            {
                // Save the current matrix
                _startMotionMatrix = getMatrix();
		startPoint = frameNode->getTransform();

                // Get the LocalToWorld and WorldToLocal matrix for this node.
                osg::NodePath nodePathToRoot;
                osgManipulator::computeNodePathToRoot(*this,nodePathToRoot);
                _localToWorld = osg::computeLocalToWorld(nodePathToRoot);
                _worldToLocal = osg::Matrix::inverse(_localToWorld);

                return true;
            }
        case osgManipulator::MotionCommand::MOVE:
            {
                // Transform the command's motion matrix into local motion matrix.
                osg::Matrix localMotionMatrix = _localToWorld * command.getWorldToLocal()
                                                * command.getMotionMatrix()
                                                * command.getLocalToWorld() * _worldToLocal;
			
                // Transform by the localMotionMatrix
		Eigen::Matrix4d mate;
		for(int i = 0; i < 4;i++) {
		    for(int j = 0; j < 4;j++) {
			mate(i,j) = localMotionMatrix(i,j);
		    }
		}
		
		mate.transposeInPlace();
		
		envire::FrameNode::TransformType trans = startPoint;

		trans.matrix() = mate * trans.matrix();
		frameNode->setTransform(trans);
		
                return true;
            }
        case osgManipulator::MotionCommand::FINISH:
            {
                return true; 
            }
        case osgManipulator::MotionCommand::NONE:
        default:
            return false;
    }
#endif

    
    
    return false;
}

bool FrameNodeSelection::receive(const osgManipulator::TranslateInLineCommand& command)
{
    return receive(static_cast<const osgManipulator::MotionCommand&>( command));
}

bool FrameNodeSelection::receive(const osgManipulator::TranslateInPlaneCommand& command)
{
    return receive(static_cast<const osgManipulator::MotionCommand&>( command));
}

bool FrameNodeSelection::receive(const osgManipulator::Scale1DCommand& command)
{
    //we do not react to scale commands
    return false;
}

bool FrameNodeSelection::receive(const osgManipulator::Scale2DCommand& command)
{
    //we do not react to scale commands
    return false;
}

bool FrameNodeSelection::receive(const osgManipulator::ScaleUniformCommand& command)
{
    //we do not react to scale commands
    return false;
}

bool FrameNodeSelection::receive(const osgManipulator::Rotate3DCommand& command)
{
    return receive(static_cast<const osgManipulator::MotionCommand&>( command));
}

}
