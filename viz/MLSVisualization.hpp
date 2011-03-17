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
	osg::Vec4 horizontalCellColor;
	osg::Vec4 verticalCellColor;
	osg::Vec4 uncertaintyColor;

	mutable osg::ref_ptr<osg::Geode> extents;

	bool showUncertainty;
	bool estimateNormals;
	bool cycleHeightColor;
};

#endif // POINTCLOUDVISUALIZATION_H
