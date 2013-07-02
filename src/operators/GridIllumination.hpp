#ifndef ENVIRE_GRIDILLUMINATION__
#define ENVIRE_GRIDILLUMINATION__

#include <envire/Core.hpp>

namespace envire
{
class GridIllumination : public Operator
{
    ENVIRONMENT_ITEM( GridIllumination )

public:
    GridIllumination();
    bool updateAll();

    void setLightSource( const base::Vector3d& ls, double diameter = 0.0 );
    void setOutputBand( const std::string& band );

private:
    base::Vector3d lightSource;
    double lightDiameter;
    std::string band;
};
}
#endif
