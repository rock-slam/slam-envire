#ifndef TRAVERSABILITYGRIDVISUALIZATION_HPP
#define TRAVERSABILITYGRIDVISUALIZATION_HPP

#include "GridVisualizationBase.hpp"

class TraversabilityGridVisualization: public envire::GridVisualizationBase
{

public:
    TraversabilityGridVisualization();
    
    virtual bool handlesItem(envire::EnvironmentItem* item) const;
    virtual void updateNode2(envire::EnvironmentItem* item, osg::Group* node) const;
    virtual void updateNode(envire::EnvironmentItem* item, osg::Group* node) const;
    virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
    
};

#endif // TRAVERSABILITYGRIDVISUALIZATION_HPP
