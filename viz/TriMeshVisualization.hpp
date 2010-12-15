#ifndef TRIMESHVISUALIZATION_H
#define TRIMESHVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"

class TriMeshVisualization : public vizkit::EnvironmentItemVisualizer
{
    public:
	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;
};

#endif // TRIMESHVISUALIZATION_H
