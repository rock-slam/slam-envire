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

void TraversabilityGrowClasses::growTerrains(TraversabilityGrid& mapIn, TraversabilityGrid& mapOut)
{
    const double width_square = pow(radius,2);
    const int 
        wx = radius / mapIn.getScaleX() + 1, 
        wy = radius / mapIn.getScaleY() + 1;
    const double 
        sx = mapIn.getScaleX(),
        sy = mapIn.getScaleY();


    TraversabilityGrid::ArrayType& trDataIn = mapIn.getGridData(TraversabilityGrid::TRAVERSABILITY);
    TraversabilityGrid::ArrayType& probDataIn = mapIn.getGridData(TraversabilityGrid::PROBABILITY);

    TraversabilityGrid::ArrayType& trDataOut = mapOut.getGridData(TraversabilityGrid::TRAVERSABILITY);
    TraversabilityGrid::ArrayType& probDataOut = mapOut.getGridData(TraversabilityGrid::PROBABILITY);

    if(trDataIn.num_elements() != trDataOut.num_elements())
        throw std::runtime_error("ObjectGrowing, input and output data have differens sizes");

    memcpy(trDataOut.data(), trDataIn.data(), sizeof(uint8_t) * trDataIn.num_elements());
    memcpy(probDataOut.data(), probDataIn.data(), sizeof(uint8_t) * probDataIn.num_elements());

    assert(trDataIn.shape()[0] == mapIn.getCellSizeY());
    assert(trDataIn.shape()[1] == mapIn.getCellSizeX());
    assert(trDataOut.shape()[0] == mapIn.getCellSizeY());
    assert(trDataOut.shape()[1] == mapIn.getCellSizeX());
    
    const std::vector<TraversabilityClass> &classes(grid->getTraversabilityClasses());
    
    int i = 0;
    for(std::vector<TraversabilityClass>::const_iterator it = classes.begin(); it != classes.end(); it++)
    {
        mapOut.setTraversabilityClass(i, *it);
        i++;
    }

    for (unsigned int y = 0; y < mapIn.getCellSizeY(); ++y)
    {
        for (unsigned int x = 0; x < mapIn.getCellSizeX(); ++x)
        {
            double curProbability = mapIn.getProbability(x, y);
            //don't grow unknown areas
            if(curProbability <= 0.0001)
                continue;
            
            uint8_t classNumber = trDataIn[y][x];
            TraversabilityClass curClass(classes[classNumber]);

            //grow all object in the radius
            for( int oy = -wy; oy <= wy; ++oy )
            {
                for( int ox = -wx; ox <= wx; ++ox )
                {
                    const int tx = x+ox;
                    const int ty = y+oy;
                    if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
                            && tx >= 0 && tx < (int)mapIn.getCellSizeX()
                            && ty >= 0 && ty < (int)mapIn.getCellSizeY() )
                    {
                        TraversabilityClass otherClass(classes[trDataOut[ty][tx]]);
                        if((mapOut.getProbability(tx, ty) <= 0.0001)
                           || (otherClass.getDrivability() > curClass.getDrivability()))
                        {
                            trDataOut[ty][tx] = classNumber;
                            mapOut.setProbability(curProbability, tx, ty);
                        }
                    }
                }
            }
        }
    }
}

bool envire::TraversabilityGrowClasses::updateAll()
{
    grid = getInput< envire::TraversabilityGrid *>();
    if (!grid)
        throw std::runtime_error("TraversabilityGrassfire: no input band set");

    gridOut = getOutput< envire::TraversabilityGrid *>();
    if (!gridOut)
        throw std::runtime_error("TraversabilityGrassfire: no output band set");

    growTerrains(*grid, *gridOut);
    
    return envire::Operator::updateAll();
}
