#include "TraversabilityGrassfire.hpp"
#include <maps/MLSGrid.hpp>

using namespace envire;
using envire::Grid;

ENVIRONMENT_ITEM_DEF( TraversabilityGrassfire );

int slopeTooHigh = 0;
int stepTooHigh = 0;
int totalCnt = 0;
int drivable = 0;

double TraversabilityGrassfire::getTraversability(SurfacePatch* from, SurfacePatch* to)
{
    totalCnt++;
    if(fabs(from->getMaxZ() - to->getMaxZ()) > config.maxStepHeight)
    {
        stepTooHigh++;
        //non traversable
        return 0.0;
    }
    
    double slope = to->getSlope();
    
    if(slope > config.maxSlope)
    {
        slopeTooHigh++;
        return 0.0;
    }
    
    drivable++;
    
    //scale to max slope as best traversability
    return 1.0 - slope / config.maxSlope;
}


void TraversabilityGrassfire::checkRecursive(size_t x, size_t y, SurfacePatch* origin)
{
    if(visited[y][x])
    {
        if(!bestPatchMap[y][x])
            return;
        
        double traversability = getTraversability(origin, bestPatchMap[y][x]);
        if(drivabilityMap[y][x] > traversability)
        {
            //worse to drive through
            drivabilityMap[y][x] = traversability;


            //-0.00001 to get rid of precision problems...
            (*trData)[y][x] = OBSTACLE + ceil((traversability - 0.00001) * config.numTraversabilityClasses);

            //TODO this would be the place for multi dimensional traversability map generation...
        }
        else
        {
            //better traversability
            //we search for worst drivability of the best matches, so we stop here and do nothing
            return;
        }
        
    }
    
    visited[y][x] = true;
    double bestTraversability = -1;
    SurfacePatch *bestMatchingPatch = NULL;
    
    MLSGrid::iterator it = mlsGrid->beginCell(x, y);
    for(; it != mlsGrid->endCell(); it++)
    {
        //ignored for now
//         MLSGrid::iterator nextPatch = it;
//         nextPatch++;
//         if(nextPatch != mlsGrid->endCell())
//         {
//             //check if the robot fits between this and the next patch
//             if(fabs(nextPatch->getMinZ() - it->getMaxZ()) < config.robotHeight)
//             {
//                 //it dosen't fit, patch is non traversable
//                 continue;
//             }
//         }
        //if there is no next patch, everything is fine, as it
        //means there is no obstacle obove the current patcht
        
        double curTraversability = getTraversability(origin, &(*it));
        if(bestTraversability < curTraversability)
        {
            bestTraversability = curTraversability;
            bestMatchingPatch =&(*it);
        }
    }
    
    if(bestMatchingPatch != NULL)
    {
        bestPatchMap[y][x] = bestMatchingPatch;
        drivabilityMap[y][x] = bestTraversability;
        
        //-0.00001 to get rid of precision problems...
        (*trData)[y][x] = OBSTACLE + ceil((bestTraversability - 0.00001) * config.numTraversabilityClasses);
        
        //recuse to surounding patches
        for(int yi = -1; yi <= 1; yi++)
        {
            for(int xi = -1; xi <= 1; xi++)
            {
                if(yi == 0 && xi == 0)
                    continue;
                
                size_t newX = x + xi;
                size_t newY = y + yi;
                if(newX < mlsGrid->getCellSizeX() && newY < mlsGrid->getCellSizeY())
                {
                    searchList.push(SearchItem(newX, newY, bestMatchingPatch));
                }
            }
        }
    } 
    else
    {
        (*trData)[y][x] = UNKNOWN;
    }
}

bool TraversabilityGrassfire::startRecursion(base::Vector3d startPos, bool searchSourunding)
{
    std::cout << "Input pos is " << startPos.transpose() << std::endl;
    size_t startX;
    size_t startY;
    if(!mlsGrid->toGrid(startPos, startX, startY, mlsGrid->getEnvironment()->getRootNode()))
        return false;

    std::cout << "Start is " << startX << " " << startY << std::endl;
    
    //make shure temp maps have correct size
    bestPatchMap.resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);
    drivabilityMap.resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);
    visited.resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);
    trData->resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);

    //fill them with defautl values
    SurfacePatch *emptyPatch = NULL;
    //Note passing directly NULL to fill makes the compiler cry....
    std::fill(bestPatchMap.data(), bestPatchMap.data() + bestPatchMap.num_elements(), emptyPatch);
    std::fill(drivabilityMap.data(), drivabilityMap.data() + drivabilityMap.num_elements(), 0);
    std::fill(visited.data(), visited.data() + visited.num_elements(), false);
    std::fill(trData->data(), trData->data() + trData->num_elements(), UNKNOWN);    
    
    double bestHeightDiff = std::numeric_limits< double >::max();
    SurfacePatch *bestMatchingPatch = NULL;

    //search the sourounding of the start pos for a start patch
    for(int i = 0; i < 10; i++)
    {
        for(int yi = -i; yi <= i; yi++)
        {
            for(int xi = -i; xi <= i; xi++)
            {
                //only check the 'outer rim'
                if(abs(yi) != i && abs(xi) != i)
                    continue;
                
                size_t newX = startX + xi;
                size_t newY = startY + yi;
                if(newX < mlsGrid->getCellSizeX() && newY < mlsGrid->getCellSizeY())
                {
                    //look for patch with best height
                    MLSGrid::iterator it = mlsGrid->beginCell(startX, startY);
                    for(; it != mlsGrid->endCell(); it++)
                    {
                        double curHeightDiff = fabs(startPos.z() - it->getMaxZ());
                        if(curHeightDiff < bestHeightDiff)
                        {
                            bestMatchingPatch = &(*it);
                            bestHeightDiff = curHeightDiff;
                        }
                    }
                }
            }
        }
        if(bestMatchingPatch)
            break;
    }

    if(!bestMatchingPatch)
    {
        //we are screwed, can't start the grassfire
        return false;
    }

    bestPatchMap[startY][startX] = bestMatchingPatch;
    double slope = bestMatchingPatch->getSlope();
    if(slope > config.maxSlope)
    {
        drivabilityMap[startY][startX] = 0;
    }
    else
    {
        drivabilityMap[startY][startX] = 1.0 - slope / config.maxSlope;
    }
    visited[startY][startX] = true;
        
    //-0.00001 to get rid of precision problems...
    (*trData)[startY][startX] = OBSTACLE + ceil((drivabilityMap[startY][startX] - 0.00001) * config.numTraversabilityClasses);
        
    //recuse to surounding patches
    for(int yi = -1; yi < 1; yi++)
    {
        for(int xi = -1; xi < 1; xi++)
        {
            if(yi == 0 && xi == 0)
                continue;
            
            size_t newX = startX + xi;
            size_t newY = startY + yi;
            if(newX < mlsGrid->getCellSizeX() && newY < mlsGrid->getCellSizeY())
            {
                searchList.push(SearchItem(newX, newY, bestMatchingPatch));
            }
        }
    }
    
    return true;
}


void TraversabilityGrassfire::setStartPosition(Eigen::Vector3d startPos)
{
    this->startPos = startPos;
}

void TraversabilityGrassfire::computeSlopeMap()
{
//     size_t maxX = mlsGrid->getCellSizeX();
//     size_t maxY = mlsGrid->getCellSizeY();
//     
//     for(size_t y = 0;y < maxY; y++)
//     {
//         for(size_t x = 0;x < maxX; x++)
//         {
//             slopeMap[y][x] = ml
//         }
//     }

}

void TraversabilityGrassfire::smoothSlopeMap()
{

}


bool envire::TraversabilityGrassfire::updateAll()
{
    slopeTooHigh = 0;
    stepTooHigh = 0;
    totalCnt = 0;
    drivable = 0;

    mlsGrid = getInput<envire::MLSGrid *>();
    if(!mlsGrid)
        throw std::runtime_error("TraversabilityGrassfire: no input band set");
    
    trGrid = getOutput< envire::TraversabilityGrid *>();
    if (!trGrid)
        throw std::runtime_error("TraversabilityGrassfire: no output band set");

    
    trData = &(trGrid->getGridData(TraversabilityGrid::TRAVERSABILITY));

    //register classes in traversability map
    trGrid->setTraversabilityClass(UNKNOWN, TraversabilityClass(1.0));
    trGrid->setTraversabilityClass(OBSTACLE, TraversabilityClass(0));
    int numClasses = config.numTraversabilityClasses;
    for(int i = 1; i <= config.numTraversabilityClasses; i++)
    {
        trGrid->setTraversabilityClass(OBSTACLE + i, TraversabilityClass(1.0 / numClasses * i));
    }

    if(!startRecursion(startPos))
    {
        std::cout << "TraversabilityGrassfire::Warning, could not start recursion" << std::endl;
        return false;
    }
    
    while(!searchList.empty())
    {
        SearchItem next = searchList.front();
        searchList.pop();
        
        checkRecursive(next.x, next.y, next.origin);
        
    }
/*    
    std::cout << "stepTooHigh " << stepTooHigh << std::endl;
    std::cout << "slopeTooHigh " << slopeTooHigh << std::endl;
    std::cout << "totalCnt " << totalCnt << std::endl;
    std::cout << "drivable " << drivable << std::endl;*/
        
    return envire::Operator::updateAll();
}
