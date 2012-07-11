#ifndef OCCUPANCYGRIDVISUALIZATION_H
#define OCCUPANCYGRIDVISUALIZATION_H 

#include "GridVisualizationBase.hpp"
#include <vector>
#include <osg/Node>

namespace envire
{
  class OccupancyGridVisualization : public GridVisualizationBase
  {
      public:
	  virtual bool handlesItem(envire::EnvironmentItem *item) const;
	  virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
          void setColor(const osg::Vec4d& color, osg::Geode* geode);
  };
}

#endif // LASERSCANVISUALIZATION_H
