#ifndef ELEVATIONGRIDVISUALIZATION_H
#define ELEVATIONGRIDVISUALIZATION_H

#include "GridVisualizationBase.hpp"
#include <vector>
#include <osg/Node>
#include <envire/maps/Grids.hpp>

namespace vizkit 
{
  class ElevationGridVisualization : public GridVisualizationBase
  {
      public:
	  virtual bool handlesItem(envire::EnvironmentItem *item) const;
	  virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
      private:
	  void loadElevationGridAsHeightMap(const osg::ref_ptr<osg::Geode> geode,
					   envire::ElevationGrid &elevation_grid,
				           const osg::ref_ptr<osg::Image> texture)const;
  };
}

#endif // LASERSCANVISUALIZATION_H
