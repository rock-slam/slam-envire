#ifndef TRAVERSABILITYGRASSFIRE_H
#define TRAVERSABILITYGRASSFIRE_H

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/MLSPatch.hpp>
#include <queue>

namespace envire {

class TraversabilityGrassfire: public envire::Operator {
        ENVIRONMENT_ITEM( TraversabilityGrassfire );

public:
    class Config
    {
    public:
        Config(): maxStepHeight(0), maxSlope(0), robotHeight(0), numTraversabilityClasses(0), numNominalMeasurements(1), outliertFilterMinMeasurements(0), outliertFilterMaxStdDev(0.0) {};
        double maxStepHeight;
        double maxSlope;
        double robotHeight;
        int numTraversabilityClasses;
        /**
         * The amount of measurements a MLS-Patch needs
         * to get a probability of 1.0
         * */
        int numNominalMeasurements;
        
        int outliertFilterMinMeasurements;
        double outliertFilterMaxStdDev;
    };
    
    virtual bool updateAll();

    void setStartPosition(Eigen::Vector3d startPos);
    void setConfig(const Config &config)
    {
        this->config = config;
    }
    
private:
    SurfacePatch *getNearestPatchWhereRobotFits(size_t x, size_t y, double height);
    void addNeightboursToSearchList(size_t x, size_t y, SurfacePatch *patch);
    
    double getStepHeight(SurfacePatch *from, SurfacePatch *to);
    void markAsObstacle(size_t x, size_t y);
    
    bool determineDrivePlane();
    
    base::Vector3d startPos;
    base::Vector2d startPos_map;
    Config config;
    envire::TraversabilityGrid *trGrid;
    TraversabilityGrid::ArrayType *trData;
    MLSGrid *mlsGrid;
    
    TraversabilityClass classUnknown;
    TraversabilityClass classObstacle;
    
    class SearchItem
    {
    public:
        SearchItem(size_t x, size_t y, envire::SurfacePatch* origin) : x(x), y(y), origin(origin)
        {
        };
        
        size_t x;
        size_t y;
        envire::SurfacePatch* origin;
    };
    
    std::queue<SearchItem> searchList;
    
    boost::multi_array<bool, 2> visited;
    boost::multi_array<envire::SurfacePatch *, 2> bestPatchMap;

    void computeTraversability();
    void setTraversability(size_t x, size_t y);
    void setProbability(size_t x, size_t y);
    void checkRecursive(size_t x, size_t y, envire::SurfacePatch* origin);
    bool determineDrivePlane(base::Vector3d startPos, bool searchSourunding = true);
    
    enum TRCLASSES
    {
        UNKNOWN = 0,
        OBSTACLE = 1,
    };
};

}
#endif // TRAVERSABILITYGRASSFIRE_H
