#ifndef FRAMENODEMANIPULATOR_H
#define FRAMENODEMANIPULATOR_H
#include <envire/Core.hpp>
#include <osg/Group>
#include <osgManipulator/TranslateAxisDragger>
#include <osgManipulator/TrackballDragger>
#include <osg/MatrixTransform>
#include <osg/Version>

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
#if OPENSCENEGRAPH_MAJOR_VERSION >= 3 
	osg::ref_ptr<osgManipulator::DraggerCallback> draggerCallback;
#endif
};

}
#endif // FRAMENODEMANIPULATOR_H
