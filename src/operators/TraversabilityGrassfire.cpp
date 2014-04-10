#include "TraversabilityGrassfire.hpp"
#include <maps/MLSGrid.hpp>

using namespace envire;
using envire::Grid;

ENVIRONMENT_ITEM_DEF( TraversabilityGrassfire );

int slopeTooHigh = 0;
int stepTooHigh = 0;
int totalCnt = 0;
int drivable = 0;

void TraversabilityGrassfire::setProbability(size_t x, size_t y)
{
    SurfacePatch *currentPatch = bestPatchMap[y][x];
    if(!currentPatch)
    {
        trGrid->setProbability(0.0, x, y);
        (*trData)[y][x] = UNKNOWN;
        return;
    }

    float numScanPoints = currentPatch->getMeasurementCount();
    
    if(numScanPoints > config.numNominalMeasurements)
    {
        trGrid->setProbability(1.0, x, y);        
    }
    else
    {
        trGrid->setProbability(numScanPoints / config.numNominalMeasurements, x, y);
    }
}


void TraversabilityGrassfire::setTraversability(size_t x, size_t y)
{
    bool debug = false;
    totalCnt++;

    SurfacePatch *currentPatch = bestPatchMap[y][x];
    if(!currentPatch)
    {
        (*trData)[y][x] = UNKNOWN;
        return;
    }
    
    base::PlaneFitting<double> fitter;
    int count = 0;
    
    double thisHeight = currentPatch->getMean() + currentPatch->getStdev();

    const double scaleX =mlsGrid->getScaleX();
    const double scaleY = mlsGrid->getScaleY();

    if(debug)
    {
        std::cout << "x " << x << " y " << y << " height " << thisHeight << " mean " << currentPatch->getMean() << " stdev " << currentPatch->getStdev() << std::endl;
    }
    
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

                SurfacePatch *neighbourPatch = bestPatchMap[newY][newX];
                if(neighbourPatch)
                {
                    count++;
                    double neighbourHeight = neighbourPatch->getMean() + neighbourPatch->getStdev();
                    
                    if(debug)
                    {
                        std::cout << "Nx " << newX << " Ny " << newY << " height " << neighbourHeight << " mean " << neighbourPatch->getMean() << " stdev " << neighbourPatch->getStdev() << std::endl;
                    }

                    if(fabs(neighbourHeight - thisHeight) > config.maxStepHeight)
                    {
                        if(debug)
                        {
                            std::cout << "Step do hight" << std::endl;
                        }
                        stepTooHigh++;
                        (*trData)[y][x] = OBSTACLE;
                        return;
                    }
                    
                    Eigen::Vector3d input(xi * scaleX, yi * scaleY, thisHeight - neighbourHeight);
                    if(debug)
                    {
                        std::cout << "Input to plane fitter " << input.transpose() << std::endl;
                    }

                    fitter.update(input);
                }
                else
                {
                    if(debug)
                    {
                        std::cout << "Nx " << newX << " Ny " << newY << " is unknown " << std::endl;
                    }
                }
            }
        }
    }
    
    fitter.update(Eigen::Vector3d(0,0,0));
            
    if (count < 5)
    {
        if(debug)
        {
            std::cout << "count to small " << count << "Setting patch to unknown " << std::endl;
        }

        (*trData)[y][x] = UNKNOWN;
        return;
    }

    Eigen::Vector3d fit(fitter.getCoeffs());
    const double divider = sqrt(fit.x() * fit.x() + fit.y() * fit.y() + 1);
    double slope = acos(1 / divider);

    if(debug)
    {
        std::cout << "slope is " << slope << std::endl;
    }

    if(slope > config.maxSlope)
    {
        if(debug)
        {
            std::cout << "slope is to hight " << std::endl;
        }

        slopeTooHigh++;
        (*trData)[y][x] = OBSTACLE;
        return;
    }

    double drivability = 1.0 - (slope / config.maxSlope); 

    //-0.00001 to get rid of precision problems...
    (*trData)[y][x] = OBSTACLE + ceil((drivability - 0.00001) * config.numTraversabilityClasses);
    
    if(debug)
    {
        std::cout << "Setting tr class " << ceil((drivability - 0.00001) * config.numTraversabilityClasses) << std::endl;
    }

    drivable++;
}

double TraversabilityGrassfire::getStepHeight(SurfacePatch* from, SurfacePatch* to)
{
    return fabs((from->getMean() + from->getStdev()) - (to->getMean() + to->getStdev()));
}

void TraversabilityGrassfire::checkRecursive(size_t x, size_t y, SurfacePatch* origin)
{
    if(visited[y][x])
    {
        return;
    }
    
    visited[y][x] = true;
    SurfacePatch *bestMatchingPatch = getNearestPatchWhereRobotFits(x, y, origin->getMean() + origin->getStdev());

    if(bestMatchingPatch)
    {
        addNeightboursToSearchList(x, y, bestMatchingPatch);
    } 
    else
    {
        bestPatchMap[y][x] = NULL;
    }
        
}

void TraversabilityGrassfire::addNeightboursToSearchList(size_t x, size_t y, SurfacePatch* patch)
{
    bestPatchMap[y][x] = patch;
    visited[y][x] = true;

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
                searchList.push(SearchItem(newX, newY, patch));
            }
        }
    }
}


bool TraversabilityGrassfire::determineDrivePlane(base::Vector3d startPos, bool searchSourunding)
{
    size_t startX;
    size_t startY;
    if(!mlsGrid->toGrid(startPos, startX, startY, mlsGrid->getEnvironment()->getRootNode()))
        return false;

    
    //make shure temp maps have correct size
    bestPatchMap.resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);
    visited.resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);
    trData->resize(boost::extents[mlsGrid->getCellSizeY()][mlsGrid->getCellSizeX()]);

    //fill them with defautl values
    SurfacePatch *emptyPatch = NULL;
    //Note passing directly NULL to fill makes the compiler cry....
    std::fill(bestPatchMap.data(), bestPatchMap.data() + bestPatchMap.num_elements(), emptyPatch);
    std::fill(visited.data(), visited.data() + visited.num_elements(), false);
    std::fill(trData->data(), trData->data() + trData->num_elements(), UNKNOWN);    
    
    double bestHeightDiff = std::numeric_limits< double >::max();
    SurfacePatch *bestMatchingPatch = NULL;

    size_t correctedStartX = startX;
    size_t correctedStartY = startY;
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
                    SurfacePatch *curPatch = getNearestPatchWhereRobotFits(newX, newY, startPos.z());
                    if(curPatch)
                    {
                        double curHeightDiff = fabs(startPos.z() - curPatch->getMean() + curPatch->getStdev());
                        if(curHeightDiff < bestHeightDiff)
                        {
                            bestMatchingPatch = curPatch;
                            bestHeightDiff = curHeightDiff;
                            correctedStartX = newX;
                            correctedStartY = newY;
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
        
    //recuse to surounding patches
    addNeightboursToSearchList(correctedStartX, correctedStartY, bestMatchingPatch);
    
    return true;
}

SurfacePatch* TraversabilityGrassfire::getNearestPatchWhereRobotFits(size_t x, size_t y, double height)
{
    SurfacePatch *bestMatchingPatch = NULL;
    double minDistance = std::numeric_limits< double >::max();
    
    MLSGrid::iterator it = mlsGrid->beginCell(x, y);
    MLSGrid::iterator itEnd = mlsGrid->endCell();
    for(; it != itEnd; it++)
    {
        bool gapTooSmall = false;
        MLSGrid::iterator hcIt= mlsGrid->beginCell(x, y);
        MLSGrid::iterator hcItEnd = mlsGrid->endCell();
        double curFloorHeight = it->getMean() + it->getStdev();
        for(; hcIt != hcItEnd; hcIt++)
        {
            //check if the robot can pass between this and the other patches
            double otherCeilingHeight = hcIt->getMean() - hcIt->getStdev();
            if((curFloorHeight < otherCeilingHeight) && otherCeilingHeight - curFloorHeight < config.robotHeight)
            {
                gapTooSmall = true;
                break;
            }
        }
        if(gapTooSmall)
            continue;
        
        double curDist = fabs(curFloorHeight - height);
        if(curDist < minDistance)
        {
            minDistance = curDist;
            bestMatchingPatch =&(*it);
        }        
    }
    
    return bestMatchingPatch;
}




void TraversabilityGrassfire::setStartPosition(Eigen::Vector3d startPos)
{
    this->startPos = startPos;
}

void TraversabilityGrassfire::computeTraversability()
{
    size_t maxX = mlsGrid->getCellSizeX();
    size_t maxY = mlsGrid->getCellSizeY();
    
    for(size_t y = 0;y < maxY; y++)
    {
        for(size_t x = 0;x < maxX; x++)
        {
            setTraversability(x, y);
            setProbability(x, y);
        }
    }
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

    if(!determineDrivePlane(startPos))
    {
        std::cout << "TraversabilityGrassfire::Warning, could not find plane robot is driving on" << std::endl;
        return false;
    }
    
    while(!searchList.empty())
    {
        SearchItem next = searchList.front();
        searchList.pop();
        
        checkRecursive(next.x, next.y, next.origin);
        
    }
    
    computeTraversability();
/*    
    std::cout << "stepTooHigh " << stepTooHigh << std::endl;
    std::cout << "slopeTooHigh " << slopeTooHigh << std::endl;
    std::cout << "totalCnt " << totalCnt << std::endl;
    std::cout << "drivable " << drivable << std::endl;*/
        
    return envire::Operator::updateAll();
}
