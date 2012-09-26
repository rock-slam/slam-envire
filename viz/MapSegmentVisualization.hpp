#ifndef ENVIRE_MAPSEGMENTVISUALIZATION_HPP__
#define ENVIRE_MAPSEGMENTVISUALIZATION_HPP__

#include "EnvironmentItemVisualizer.hpp"
#include <osg/Geometry>

namespace envire
{
class MapSegmentVisualization : public EnvironmentItemVisualizer
{
    Q_OBJECT
    
    public:
	MapSegmentVisualization();

	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;

    protected:
};
}

#endif 
