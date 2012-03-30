#ifndef MLSVISUALIZATION_H
#define MLSVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>

class MLSVisualization : public vizkit::EnvironmentItemVisualizer
{
    Q_OBJECT

    Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)
    Q_PROPERTY(bool show_negative READ isNegativeShown WRITE setShowNegative)
    Q_PROPERTY(bool estimate_normals READ areNormalsEstimated WRITE setEstimateNormals)
    Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
    Q_PROPERTY(QColor horizontal_cell_color READ getHorizontalCellColor WRITE setHorizontalCellColor)
    Q_PROPERTY(QColor vertical_cell_color READ getVerticalCellColor WRITE setVerticalCellColor)
    Q_PROPERTY(QColor negative_cell_color READ getNegativeCellColor WRITE setNegativeCellColor)
    Q_PROPERTY(QColor uncertainty_color READ getUncertaintyColor WRITE setUncertaintyColor)
    
    public:
	MLSVisualization();
        ~MLSVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;
        
    public slots:
        bool isUncertaintyShown() const;
        void setShowUncertainty(bool enabled);
        bool isNegativeShown() const;
        void setShowNegative(bool enabled);
        bool areNormalsEstimated() const;
        void setEstimateNormals(bool enabled);
        bool isHeightColorCycled() const;
        void setCycleHeightColor(bool enabled);
        QColor getHorizontalCellColor() const;
        void setHorizontalCellColor(QColor color);
        QColor getVerticalCellColor() const;
        void setVerticalCellColor(QColor color);
        QColor getNegativeCellColor() const;
        void setNegativeCellColor(QColor color);
        QColor getUncertaintyColor() const;
        void setUncertaintyColor(QColor color);

    protected:
	osg::Vec4 horizontalCellColor;
	osg::Vec4 verticalCellColor;
	osg::Vec4 negativeCellColor;
	osg::Vec4 uncertaintyColor;

	mutable osg::ref_ptr<osg::Geode> extents;

	bool showUncertainty;
	bool showNegative;
	bool estimateNormals;
	bool cycleHeightColor;
};

#endif // POINTCLOUDVISUALIZATION_H
