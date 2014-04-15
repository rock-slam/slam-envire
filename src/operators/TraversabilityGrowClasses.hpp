#ifndef TRAVERSABILITYGROWCLASSES_H
#define TRAVERSABILITYGROWCLASSES_H

#include <envire/maps/TraversabilityGrid.hpp>

namespace envire {
    
class TraversabilityGrowClasses : public envire::Operator 
{
    ENVIRONMENT_ITEM( TraversabilityGrowClasses );
public:
    virtual bool updateAll();
    
    void setRadius(double radius);
    
    void setTraversabilityGrid(TraversabilityGrid *grid);
private:
    void growTerrains(envire::TraversabilityGrid& mapIn, envire::TraversabilityGrid& mapOut);

    TraversabilityGrid *grid;
    TraversabilityGrid *gridOut;
    double radius;
};

}
#endif // TRAVERSABILITYGROWCLASSES_H
