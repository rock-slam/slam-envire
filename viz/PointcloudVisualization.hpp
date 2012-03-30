#ifndef POINTCLOUDVISUALIZATION_H
#define POINTCLOUDVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>

class PointcloudVisualization : public vizkit::EnvironmentItemVisualizer
{
    Q_OBJECT
    
    Q_PROPERTY(bool show_normals READ areNormalsShown WRITE setShowNormals)
    Q_PROPERTY(bool show_features READ areFeaturesShown WRITE setShowFeatures)
    Q_PROPERTY(bool color_cycling READ isColorCycled WRITE setColorCycling)
    Q_PROPERTY(double normal_scaling READ getNormalScaling WRITE setNormalScaling)
    Q_PROPERTY(QColor normal_color READ getNormalColor WRITE setNormalColor)
    Q_PROPERTY(QColor vertex_color READ getVertexColor WRITE setVertexColor)
    
    public:
	PointcloudVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;

    public slots:
        bool areNormalsShown() const;
        void setShowNormals(bool enabled);
        bool areFeaturesShown() const;
        void setShowFeatures(bool enabled);
        bool isColorCycled() const;
        void setColorCycling(bool enabled);
        double getNormalScaling() const;
        void setNormalScaling(double scaling);
        QColor getNormalColor() const;
        void setNormalColor(QColor color);
        QColor getVertexColor() const;
        void setVertexColor(QColor color);

    protected:
	osg::Vec4 vertexColor;
	osg::Vec4 normalColor;
	double normalScaling;
	bool showNormals;
	bool showFeatures;
	bool colorCycling;

};

#endif // POINTCLOUDVISUALIZATION_H
