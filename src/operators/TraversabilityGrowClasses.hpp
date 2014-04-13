#ifndef TRAVERSABILITYGROWCLASSES_H
#define TRAVERSABILITYGROWCLASSES_H

#include <envire/operators/ObjectGrowing.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

namespace envire {
    
class TraversabilityGrowClasses : public ObjectGrowing<uint8_t>, public envire::Operator 
{
        ENVIRONMENT_ITEM( TraversabilityGrowClasses );
public:
    virtual bool updateAll();
    
    void setRadius(double radius);
    
    void setTraversabilityGrid(TraversabilityGrid *grid);
private:
    TraversabilityGrid *grid;
    TraversabilityGrid *gridOut;
    double radius;
};

}
#endif // TRAVERSABILITYGROWCLASSES_H
