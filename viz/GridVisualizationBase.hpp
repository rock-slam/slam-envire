#ifndef GRIDVISUALIZATIONBASE_H
#define GRIDVISUALIZATIONBASE_H

#include "EnvironmentItemVisualizer.hpp"
#include <vector>
#include <osg/Node>
#include <envire/maps/GridBase.hpp>
// Workaround MOC not being able to expand macros properly
#ifndef Q_MOC_RUN
#include <boost/function.hpp>
#endif

namespace envire {

    
class GridVisualizationBase : public EnvironmentItemVisualizer
{
    public:
	virtual bool handlesItem(envire::EnvironmentItem *item) const = 0;
	virtual osg::Group *getNodeForItem(envire::EnvironmentItem *item) const;
	virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const = 0;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const  {};
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const  {};

	class Color
	{
	public:
	    unsigned char r;
	    unsigned char g;
	    unsigned char b;
	};
	
    protected:
	void loadImageAsRectangle(osg::ref_ptr<osg::Geode> geode,
			          osg::ref_ptr<osg::Image> image,
			          float pos_x1,float pos_y1,float pos_x2,float pos_y2)const;
				  
	void showGridAsImage(osg::ref_ptr<osg::Geode> geode, envire::GridBase *grid, boost::function<bool (int x, int y, Color &ret)> colorForGridCoordinate) const;
};

}

#endif // LASERSCANVISUALIZATION_H
