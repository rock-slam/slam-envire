#ifndef MLSVISUALIZATION_H
#define MLSVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>
#include <osg/Geode>

class MLSVisualization : public vizkit::EnvironmentItemVisualizer
{
    public:
        enum colorModeType{HEIGHT,CELL};

	MLSVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;

        void setColorMode(colorModeType value);
        colorModeType getColorMode();

        void showUncertainty(bool value);
        bool isUncertaintyEnabled();

        void estimateNormals(bool value);
        bool isEstimateNormalsEnabled();

    protected:
	osg::Vec4 horizontalCellColor;
	osg::Vec4 verticalCellColor;
	osg::Vec4 uncertaintyColor;

	mutable osg::ref_ptr<osg::Geode> extents;
	bool show_uncertainty;
	bool estimate_normals;
        colorModeType color_mode;
};

#endif // POINTCLOUDVISUALIZATION_H
