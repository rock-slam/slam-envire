#ifndef GRIDVISUALIZATIONBASE_H
#define GRIDVISUALIZATIONBASE_H

#include "EnvironmentItemVisualizer.hpp"
#include <vector>
#include <osg/Node>

namespace envire {

    
class GridVisualizationBase : public EnvironmentItemVisualizer
{
    public:
	virtual bool handlesItem(envire::EnvironmentItem *item) const = 0;
	virtual osg::Group *getNodeForItem(envire::EnvironmentItem *item) const;
	virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const = 0;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const  {};
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const  {};

    protected:
	void loadImageAsRectangle(osg::ref_ptr<osg::Geode> geode,
			          osg::ref_ptr<osg::Image> image,
			          float pos_x1,float pos_y1,float pos_x2,float pos_y2)const;
};

}

#endif // LASERSCANVISUALIZATION_H
