#ifndef ELEVATIONGRIDVISUALIZATION_H
#define ELEVATIONGRIDVISUALIZATION_H

#include "GridVisualizationBase.hpp"
#include <vector>
#include <osg/Node>
// Workaround MOC not being able to expand macros properly
#ifndef Q_MOC_RUN
#include <envire/maps/ElevationGrid.hpp>
#endif

namespace envire 
{
  class ElevationGridVisualization : public GridVisualizationBase
  {
      Q_OBJECT
      
      Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
      
      public:
        ElevationGridVisualization();
          
	  virtual bool handlesItem(envire::EnvironmentItem *item) const;
	  virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
          
      public slots:
        bool isHeightColorCycled() const;
        void setCycleHeightColor(bool enabled);
          
      private:
	  void loadElevationGridAsHeightMap(const osg::ref_ptr<osg::Geode> geode,
					   envire::ElevationGrid &elevation_grid,
				           const osg::ref_ptr<osg::Image> texture,
                                           double z_offset
                                           )const;

        bool cycleHeightColor;
  };
}

#endif // LASERSCANVISUALIZATION_H
