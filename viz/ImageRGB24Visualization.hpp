#ifndef IMAGERGB24VISUALIZATION_H
#define IMAGERGB24VISUALIZATION_H

#include "GridVisualizationBase.hpp"
#include <vector>
#include <osg/Node>

namespace envire 
{
  class ImageRGB24Visualization : public GridVisualizationBase
  {
      public:
	  virtual bool handlesItem(envire::EnvironmentItem *item) const;
	  virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
  };
}

#endif // LASERSCANVISUALIZATION_H
