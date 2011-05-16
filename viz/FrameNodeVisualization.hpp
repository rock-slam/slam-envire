#ifndef FRAMENODEVISUALIZATION_H
#define FRAMENODEVISUALIZATION_H
#include "EnvironmentItemVisualizer.hpp"

namespace vizkit {

class FrameNodeVisualization: public vizkit::EnvironmentItemVisualizer
{
    public:
	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
        virtual void hideNode(envire::EnvironmentItem *item, osg::Group *group) const;
        virtual void unHideNode(envire::EnvironmentItem *item, osg::Group *group) const;
};

};
#endif // FRAMENODEVISUALIZATION_H
