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
        Config(): maxStepHeight(0), maxSlope(0), robotHeight(0), numTraversabilityClasses(0) {};
        double maxStepHeight;
        double maxSlope;
        double robotHeight;
        int numTraversabilityClasses;
    };
    
    virtual bool updateAll();

    void setStartPosition(Eigen::Vector3d startPos);
    void setConfig(const Config &config)
    {
        this->config = config;
    }
    
private:
    void computeSlopeMap();
    void smoothSlopeMap();
    
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
    boost::multi_array<double, 2> drivabilityMap;
    boost::multi_array<double, 2> slopeMap;
    boost::multi_array<double, 2> smoothedSlopeMap;

    double getTraversability(envire::SurfacePatch* from, envire::SurfacePatch* to);
    void checkRecursive(size_t x, size_t y, envire::SurfacePatch* origin);
    bool startRecursion(base::Vector3d startPos, bool searchSourunding = true);
    
    enum TRCLASSES
    {
        UNKNOWN = 0,
        OBSTACLE = 1,
    };
};

}
#endif // TRAVERSABILITYGRASSFIRE_H
