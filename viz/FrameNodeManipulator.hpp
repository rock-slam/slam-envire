#ifndef FRAMENODEMANIPULATOR_H
#define FRAMENODEMANIPULATOR_H
#include <envire/Core.hpp>
#include <osg/Group>
#include <osgManipulator/TranslateAxisDragger>
#include <osgManipulator/TrackballDragger>
#include <osg/MatrixTransform>

namespace envire {

class FrameNodeManipulator
{
    public:
	FrameNodeManipulator( envire::EnvironmentItem* item, osg::Group* parentNode );
	~FrameNodeManipulator();
	
    private:
	envire::FrameNode *fr;
	osg::Group *parentNode;

	osg::ref_ptr<osg::Node> dragger;
	osg::ref_ptr<osg::MatrixTransform> selection;
	osg::ref_ptr<osgManipulator::DraggerCallback> draggerCallback;
};

}
#endif // FRAMENODEMANIPULATOR_H
