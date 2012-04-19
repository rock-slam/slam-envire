#ifndef LASERSCANVISUALIZATION_H
#define LASERSCANVISUALIZATION_H

#include "EnvironmentItemVisualizer.hpp"
#include <envire/maps/LaserScan.hpp>
#include <vector>
#include <osg/Node>

namespace envire {

    
class LaserScanVisualization : public envire::EnvironmentItemVisualizer
{
    public:
	virtual bool handlesItem(envire::EnvironmentItem *item) const;
	virtual osg::Group *getNodeForItem(envire::EnvironmentItem *item) const;
	virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
    private:
	osg::Node *getNodeForLine(const envire::LaserScan::scanline_t& line, double stepSize, double startingAngle) const;
};

}

#endif // LASERSCANVISUALIZATION_H
