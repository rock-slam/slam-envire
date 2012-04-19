#ifndef FRAMENODEMANIPULATOR_H
#define FRAMENODEMANIPULATOR_H
#include <envire/Core.hpp>
#include <osg/Group>
#include <osgManipulator/Selection>
#include <osgManipulator/Dragger>
#include <osgManipulator/CommandManager>
#include "FrameNodeSelection.hpp"
#include <osgManipulator/TranslateAxisDragger>
#include <osgManipulator/TrackballDragger>
#include <osg/PositionAttitudeTransform>

namespace envire {

class FrameNodeManipulator
{
    public:
	FrameNodeManipulator(envire::EnvironmentItem* item, osg::Group* parentNode, osg::Group* group);
	~FrameNodeManipulator();
	
    private:
	envire::FrameNode *fr;
	osg::PositionAttitudeTransform *transform;
	osg::Group *parentNode;
	osgManipulator::CommandManager *cm;
	osg::ref_ptr<osgManipulator::TranslateAxisDragger> translateDragger;
	osg::ref_ptr<osgManipulator::TrackballDragger> rotateDragger;
	osg::ref_ptr<osgManipulator::Selection> selectionForTrackball;
	osg::ref_ptr<osgManipulator::Selection> selectionForFrameNode;
};

}
#endif // FRAMENODEMANIPULATOR_H
