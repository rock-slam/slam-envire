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
	bool setShowNormals( bool showNormals );
	bool setShowFeatures( bool showFeatures );

    protected:
	osg::Vec4 vertexColor;
	osg::Vec4 normalColor;
	double normalScaling;
	bool showNormals;
	bool showFeatures;

};

#endif // POINTCLOUDVISUALIZATION_H
