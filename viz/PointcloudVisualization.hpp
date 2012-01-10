#ifndef POINTCLOUDVISUALIZATION_H
#define POINTCLOUDVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>

class PointcloudVisualization : public vizkit::EnvironmentItemVisualizer
{
    public:
	PointcloudVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;

    protected:
	/** will set the color for all pointclouds of this visualizer. 
	 * @todo make this changeable per item
	 */
	bool setDefaultColor( const osg::Vec4& color );
	void setShowNormals( bool showNormals ){this->showNormals = showNormals;updateVisualizedItems();};
	void setShowFeatures( bool showFeatures ){this->showFeatures = showFeatures;updateVisualizedItems();};

        bool isNormalsEnabled(){return showNormals;};
        bool isFeaturesEnabled(){return showFeatures;};

    protected:
	osg::Vec4 vertexColor;
	osg::Vec4 normalColor;
	double normalScaling;
	bool showNormals;
	bool showFeatures;
	bool colorCycling;

};

#endif // POINTCLOUDVISUALIZATION_H
