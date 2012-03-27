#ifndef __ENVIRE_OPERATORS_MERGEMLS__
#define __ENVIRE_OPERATORS_MERGEMLS__

#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>

namespace envire 
{

class MergeMLS : public Operator
{
    ENVIRONMENT_ITEM( MergeMLS )

public:
    MergeMLS() {};

    bool updateAll();
};
}
#endif
