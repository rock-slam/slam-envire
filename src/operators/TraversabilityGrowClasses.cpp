#include "TraversabilityGrowClasses.hpp"

using namespace envire;

ENVIRONMENT_ITEM_DEF( TraversabilityGrowClasses );

void envire::TraversabilityGrowClasses::setRadius(double radius)
{
    this->radius = radius;
}

void envire::TraversabilityGrowClasses::setTraversabilityGrid(envire::TraversabilityGrid* grid)
{
    this->grid = grid;
}

bool envire::TraversabilityGrowClasses::updateAll()
{
    grid = getInput< envire::TraversabilityGrid *>();
    if (!grid)
        throw std::runtime_error("TraversabilityGrassfire: no input band set");

    gridOut = getOutput< envire::TraversabilityGrid *>();
    if (!gridOut)
        throw std::runtime_error("TraversabilityGrassfire: no output band set");

    const std::vector<TraversabilityClass> &classes(grid->getTraversabilityClasses());
    GrowthPolicy<uint8_t> policy(classes.size());
        
    size_t outerCnt = 0;
    for(std::vector<TraversabilityClass>::const_iterator it = classes.begin(); it != classes.end(); it++)
    {
        size_t innerCnt = 0;
        for(std::vector<TraversabilityClass>::const_iterator it2 = classes.begin(); it2 != classes.end(); it2++)
        {
            policy.setBigger(outerCnt, innerCnt, it->getDrivability() < it2->getDrivability());
            innerCnt++;
        }
        gridOut->setTraversabilityClass(outerCnt, *it);

        outerCnt++;
    }
    
    growObjects(policy, *grid, *gridOut, TraversabilityGrid::TRAVERSABILITY, radius);
    
    return envire::Operator::updateAll();
}
