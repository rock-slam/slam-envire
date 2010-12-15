#ifndef MLSVISUALIZATION_H
#define MLSVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>

class MLSVisualization : public vizkit::EnvironmentItemVisualizer
{
    public:
	MLSVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;

    protected:
	osg::Vec4 vertexColor;
	bool showUncertainty;
};

#endif // POINTCLOUDVISUALIZATION_H
