#ifndef ENVIRE_GRIDVISUALIZATION_HPP
#define ENVIRE_GRIDVISUALIZATION_HPP

#include "GridVisualizationBase.hpp"

namespace envire 
{
    class GridVisualization : public GridVisualizationBase
    {
    public:
        virtual bool handlesItem(envire::EnvironmentItem *item) const;
        virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;
    };
}

#endif 
