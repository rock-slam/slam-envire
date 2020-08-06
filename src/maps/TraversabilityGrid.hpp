#ifndef ENVIRE_TRAVERSABILITYGRID_H
#define ENVIRE_TRAVERSABILITYGRID_H

#include <envire/maps/Grid.hpp>
#include <base/samples/DistanceImage.hpp>
#include <boost/function.hpp>

namespace envire
{

class TraversabilityClass
{
public:

    /**
     * Default constructor.
     * Drivability must be given in the rande of [0,1] (0 - 100%)
     * */
    TraversabilityClass(double drivability) : drivability(drivability)
    {
        if(drivability < 0 || drivability >= 1.000001)
        {
            throw std::runtime_error("TraversabilityClass: Error, drivability must be in range [0,1]");
        }
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

/**
 * A Helper class for computing statistics in the Traversability grid.
 * */
class TraversabilityStatistic
{
public:
    TraversabilityStatistic() : countTotal(0), highestTraversabilityClass(0)
    {
        minDistance.resize(std::numeric_limits<uint8_t>::max(), std::numeric_limits<double>::max());
        counts.resize(std::numeric_limits<uint8_t>::max());
    }

    /**
     * Adds a measurment of a given klass with a given dist to a center
     * to the statistic.
     * The center is a point that is defined outside of this class.
     *
     * @arg klass : A integer that represents a TraversabilityClass
     * @arg distToCenter : Distance to a point (defined outside)
     *
     * */
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
    static const std::string PROBABILITY;
private:
    const static std::vector<std::string> bands;
    std::vector<TraversabilityClass> traversabilityClasses;
    ArrayType *probabilityArray;
    ArrayType *traversabilityArray;

    void probabilityCallback(size_t x, size_t y, double &worst) const;
    void setProbabilityArray() const;
    void setTraversabilityArray() const;
    void setProbabilityArray();
    void setTraversabilityArray();
public:
    TraversabilityGrid() : Grid<uint8_t>(), probabilityArray(NULL), traversabilityArray(NULL)
    {
    };
    TraversabilityGrid(size_t cellSizeX, size_t cellSizeY,
                        double scalex, double scaley,
                        double offsetx = 0.0, double offsety = 0.0,
                        std::string const& id = Environment::ITEM_NOT_ATTACHED):Grid<uint8_t>::Grid(cellSizeX,cellSizeY,scalex,scaley,offsetx, offsety, id),
                        probabilityArray(NULL), traversabilityArray(NULL)
    {
    };

    ~TraversabilityGrid(){};

    TraversabilityGrid &operator=(const TraversabilityGrid &other);

    void setTraversabilityAndProbability(uint8_t klass, double probability, size_t x, size_t y);

    /**
     * Sets the traversability of a grid cell to a given klass.
     * klass must be registered before by using setTraversabilityClass.
     *
     * @arg klass the integer representing the current traversability class.
     * @arg x X-Coordinate of the position
     * @arg y Y-Coordinate of the position
     * */
    void setTraversability(uint8_t klass, size_t x, size_t y);
    const TraversabilityClass &getTraversability(size_t x, size_t y) const;

    /**
     * Registeres a TraversabilityClass as a value in the grid.
     *
     * The Grid can only save values from 0-256. To associate
     * the grid values with actuall drivability values one must
     * register a TraversabilityClass for each used value.
     * */
    void setTraversabilityClass(uint8_t num, const TraversabilityClass &klass);

    /**
     * Registeres a TraversabilityClass at the grid
     *
     * @arg retId if sccessfull, the id inside of the grif for the klass is returned.
     * */
    bool registerNewTraversabilityClass(uint8_t &retId, const TraversabilityClass &klass);


    const TraversabilityClass &getTraversabilityClass(uint8_t klass) const;
    const std::vector<TraversabilityClass> &getTraversabilityClasses() const
    {
        return traversabilityClasses;
    }

    /**
     * Sets the probability of the registered TraversabilityClass
     * for a given point in the map.
     * */
    void setProbability(double probability, size_t x, size_t y);
    double getProbability(size_t x, size_t y) const;
    double getWorstProbabilityInRectangle(const base::Pose2D &pose, double sizeX, double sizeY) const;

    /**
     * Computes the statistic for an oriented rectangle in the grid.
     * The center and orientation of the rectangle is given by the
     * parameter pose.
     * @arg pose Center and Orientation of the rectangle
     * @arg sizeX Size in X of the rectangle (before orienting)
     * @arg sizeY Size in Y of the rectangle (before orienting)
     * @arg innerStatistic resulting statistic
     * */
    void computeStatistic(const base::Pose2D& pose, double sizeX, double sizeY,  envire::TraversabilityStatistic& innerStatistic) const;
    void computeStatistic(const base::Pose2D &pose, double sizeX, double sizeY, double borderWidth, TraversabilityStatistic &innerStatistic, TraversabilityStatistic &outerStatistic) const;

    const TraversabilityClass &getWorstTraversabilityClassInRectangle(const base::Pose2D &pose, double sizeX, double sizeY) const;

    virtual void serialize(Serialization& so);
    virtual void unserialize(Serialization& so);

    virtual const std::vector<std::string>& getBands() const {return bands;};
};

}
#endif // TRAVERSABILITYGRID_H
