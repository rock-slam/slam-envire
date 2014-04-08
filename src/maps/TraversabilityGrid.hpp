#ifndef ENVIRE_TRAVERSABILITYGRID_H
#define ENVIRE_TRAVERSABILITYGRID_H

#include <envire/maps/Grid.hpp>
#include <base/samples/distance_image.h>
#include <boost/function.hpp>

namespace envire
{  

class TraversabilityClass
{
public:
    TraversabilityClass(double drivability) : drivability(drivability) 
    {
        assert(drivability <= 1.000001);
    }

    TraversabilityClass() : drivability(-1) 
    {
    }

    /**
     * Return whether this class is defined
     * or if it is an empty placeholder
     * */
    bool isSet() const {
        return drivability < 0;
    }
    
    /**
     * Returns whether the terrain is drivable
     * */
    bool isTraversable() const {
        return drivability > 0;
    };

    /**
     * Returns a value in the interval [0, 1].
     * Zero means not drivable at all and
     * one means perfect ground for driving.
     * */
    double getDrivability() const
    {
        if(drivability < 0)
            return 0;
        
        return drivability;
    };
    
private:
    double drivability;
};
    
class TraversabilityStatistic
{
public:
    TraversabilityStatistic() : countTotal(0), highestTraversabilityClass(0)
    {
        minDistance.resize(std::numeric_limits<uint8_t>::max(), std::numeric_limits<double>::max());
        counts.resize(std::numeric_limits<uint8_t>::max());
    }

    void addMeasurement(uint8_t klass, double distToCenter)
    {
        countTotal++;
        highestTraversabilityClass = std::max(highestTraversabilityClass, klass);
        minDistance[klass] = std::min(distToCenter, minDistance[klass]);
        counts[klass]++;
    }
    
    void getStatisticForClass(uint8_t klass, double &minDistToCenter, size_t &count) const
    {
        minDistToCenter = minDistance[klass];
        count = counts[klass];
    }

    size_t getClassCount(uint8_t klass) const
    {
        return counts[klass];
    }

    size_t getTotalCount() const
    {
        return countTotal;
    }
    
    uint8_t getHighestTraversabilityClass() const
    {
        return highestTraversabilityClass;
    }
    
private:
    ///total count of all classes that are covered by this statistic 
    size_t countTotal;
    
    uint8_t highestTraversabilityClass;
    
    ///contains the min distance to center of every class
    std::vector<double> minDistance;
    
    ///contains the amount of occurences of every class
    std::vector<size_t> counts;
};
    
class TraversabilityGrid : public Grid<uint8_t>
{
    ENVIRONMENT_ITEM( TraversabilityGrid )
public:
    static const std::string TRAVERSABILITY;
private:
    const static std::vector<std::string> &bands;
    std::vector<TraversabilityClass> traversabilityClasses;
public:
    TraversabilityGrid() : Grid<uint8_t>() 
    {
        traversabilityClasses.resize(std::numeric_limits<uint8_t>::max());
    };
    TraversabilityGrid(size_t cellSizeX, size_t cellSizeY, 
                        double scalex, double scaley, 
                        double offsetx = 0.0, double offsety = 0.0,
                        std::string const& id = Environment::ITEM_NOT_ATTACHED):Grid<uint8_t>::Grid(cellSizeX,cellSizeY,scalex,scaley,offsetx, offsety, id)
    {
        traversabilityClasses.resize(std::numeric_limits<uint8_t>::max());
    };
    
    ~TraversabilityGrid(){};
    
    void setTraversabilityClass(uint8_t num, const TraversabilityClass &klass);
    const TraversabilityClass &getTraversabilityClass(uint8_t klass) const;
    const std::vector<TraversabilityClass> &getTraversabilityClasses() const 
    {
        return traversabilityClasses;
    }
    
    void computeStatistic(const base::Pose2D& pose, double sizeX, double sizeY,  envire::TraversabilityStatistic& innerStatistic) const;
    void computeStatistic(const base::Pose2D &pose, double sizeX, double sizeY, double borderWidth, TraversabilityStatistic &innerStatistic, TraversabilityStatistic &outerStatistic) const;

    const TraversabilityClass &getWorstTraversabilityClassInRectangle(const base::Pose2D &pose, double sizeX, double sizeY) const;
    
    virtual void serialize(Serialization& so);
    virtual void unserialize(Serialization& so);
    
    virtual const std::vector<std::string>& getBands() const {return bands;};
};

}
#endif // TRAVERSABILITYGRID_H
