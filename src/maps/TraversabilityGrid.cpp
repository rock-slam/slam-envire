#include "TraversabilityGrid.hpp"
#include <boost/bind.hpp>
#include <tools/RadialLookUpTable.hpp>
#include <tools/BoxLookUpTable.hpp>
#include <Eigen/Geometry>

using namespace envire;
using namespace Eigen;

ENVIRONMENT_ITEM_DEF( TraversabilityGrid )
const std::string TraversabilityGrid::TRAVERSABILITY = "traversability";
const std::string TraversabilityGrid::PROBABILITY = "probability";
const std::vector<std::string> TraversabilityGrid::bands = { TraversabilityGrid::TRAVERSABILITY, TraversabilityGrid::PROBABILITY };


class StatisticHelper
{
    static RadialLookUpTable *lut;
    static BoxLookUpTable *boxLut;
    const base::Pose2D &pose;
    Eigen::Rotation2D<double> inverseOrientation;
    const TraversabilityGrid &grid;
    const TraversabilityGrid::ArrayType &gridData;
    TraversabilityStatistic *innerStats;
    TraversabilityStatistic *outerStats;
    double scaleX;
    double scaleY;
    size_t xCenter;
    size_t yCenter;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StatisticHelper(const base::Pose2D &pose, const TraversabilityGrid &grid, double sizeX, double sizeY, double borderWidth) : pose(pose),
    inverseOrientation(Eigen::Rotation2D<double>(pose.orientation).inverse()), grid(grid), gridData(grid.getGridData(TraversabilityGrid::TRAVERSABILITY))
    , scaleX(grid.getScaleX()), scaleY(grid.getScaleY())
    {
        if(!lut)
            lut = new RadialLookUpTable();
        lut->recompute(grid.getScaleX(), std::max(sizeX, sizeY));

        if(!boxLut)
            boxLut = new BoxLookUpTable();
        //note the scale should be higher than the grid scale because
        //of the roation. Else we get aliasing problems.
        boxLut->recompute(scaleX / 10.0, sizeX, sizeY, borderWidth * 2);

        grid.toGrid(pose.position.x(), pose.position.y(), xCenter, yCenter);
    }

    void addInnerVal(size_t x, size_t y)
    {
        innerStats->addMeasurement(gridData[y][x], lut->getDistance(x-xCenter, y-yCenter));
    }

    void addOuterVal(size_t x, size_t y)
    {
        Vector2d pos_map;
        grid.fromGrid(x, y, pos_map.x(), pos_map.y());
        Vector2d posAligned = inverseOrientation * (pos_map - pose.position);

        outerStats->addMeasurement(gridData[y][x], boxLut->getDistanceToBox(posAligned.x(), posAligned.y()));
    }

    void setInnerStatistic(TraversabilityStatistic *innerStat)
    {
        innerStats = innerStat;
    }
    void setOuterStatistic(TraversabilityStatistic *outerStat)
    {
        outerStats = outerStat;
    }
};

RadialLookUpTable *StatisticHelper::lut;
BoxLookUpTable *StatisticHelper::boxLut;

void addVal(size_t x, size_t y, std::vector<uint8_t> &stats, const TraversabilityGrid::ArrayType &gridData)
{
    stats[gridData[y][x]]++;
}

void TraversabilityGrid::computeStatistic(const base::Pose2D &pose, double sizeX, double sizeY, TraversabilityStatistic &innerStatistic) const
{
    StatisticHelper helper(pose, *this, sizeX, sizeY, 0);
    helper.setInnerStatistic(&innerStatistic);
    forEachInRectangle(pose, sizeX, sizeY, boost::bind( &StatisticHelper::addInnerVal, &helper,  _1, _2));
}

void TraversabilityGrid::computeStatistic(const base::Pose2D &pose, double sizeX, double sizeY, double borderWidth, TraversabilityStatistic &innerStatistic, TraversabilityStatistic &outerStatistic) const
{
    StatisticHelper helper(pose, *this, sizeX, sizeY, borderWidth);
    helper.setInnerStatistic(&innerStatistic);
    helper.setOuterStatistic(&outerStatistic);
    forEachInRectangles(pose, sizeX, sizeY, boost::bind( &StatisticHelper::addInnerVal, &helper,  _1, _2),
                        sizeX + borderWidth, sizeY + borderWidth, boost::bind( &StatisticHelper::addOuterVal, &helper,  _1, _2));
}

const TraversabilityClass& TraversabilityGrid::getWorstTraversabilityClassInRectangle(const base::Pose2D& pose, double sizeX, double sizeY) const
{
    double curDrivability = std::numeric_limits< double >::max();
    int curClass = -1;

    TraversabilityStatistic innerStatistic;
    computeStatistic(pose, sizeX, sizeY, innerStatistic);

    for(uint8_t i = 0; i <= innerStatistic.getHighestTraversabilityClass(); i++)
    {
        size_t count;
        double dist;
        innerStatistic.getStatisticForClass(i, dist, count);
        if(count)
        {
            const TraversabilityClass &klass(getTraversabilityClass(i));
            if(curDrivability > klass.getDrivability())
            {
                curClass = i;
                curDrivability = klass.getDrivability();
                //can't get worse than not traversable
                if(!klass.isTraversable())
                    return klass;
            }
        }
    }

    if(curClass < 0)
    {
        std::cout << "Pose " << pose.position.transpose() << " sizeY " << sizeY << " sizeX " << sizeX << " Total Count " << innerStatistic.getTotalCount() << std::endl;
        for(uint8_t i = 0; i <= innerStatistic.getHighestTraversabilityClass(); i++)
        {
            size_t count;
            double dist;
            innerStatistic.getStatisticForClass(i, dist, count);
            const TraversabilityClass &klass(getTraversabilityClass(i));
            std::cout << "Class " << i << " drivability " << klass.getDrivability() << " count " << count << std::endl;
        }
        throw std::runtime_error("TraversabilityGrid::Error, terrain class could not be identified");
    }

    return getTraversabilityClass(curClass);
}

void TraversabilityGrid::probabilityCallback(size_t x, size_t y, double& worst) const
{
    double curProbabilty = getProbability(x, y);
    if(worst > curProbabilty)
        worst = curProbabilty;
}

double TraversabilityGrid::getWorstProbabilityInRectangle(const base::Pose2D& pose, double sizeX, double sizeY) const
{
    double ret = 1.0;
    forEachInRectangle(pose, sizeX, sizeY, boost::bind(&TraversabilityGrid::probabilityCallback, this, _1, _2, boost::ref(ret)));
    return ret;
}

void TraversabilityGrid::setTraversabilityAndProbability(uint8_t klass, double probability, size_t x, size_t y)
{
    setProbability(probability, x, y);
    setTraversability(klass, x, y);
}

void TraversabilityGrid::setTraversability(uint8_t klass, size_t x, size_t y)
{
    setTraversabilityArray();
    (*traversabilityArray)[y][x] = klass;
}

const TraversabilityClass& TraversabilityGrid::getTraversability(size_t x, size_t y) const
{
    setTraversabilityArray();
    return traversabilityClasses[(*traversabilityArray)[y][x]];
}

bool TraversabilityGrid::registerNewTraversabilityClass(uint8_t& retId, const TraversabilityClass& klass)
{
    if(traversabilityClasses.size() >= std::numeric_limits< uint8_t >::max())
        return false;

    retId = traversabilityClasses.size() + 1;
    setTraversabilityClass(retId, klass);
    return true;
}

void TraversabilityGrid::setTraversabilityClass(uint8_t num, const TraversabilityClass& klass)
{
    if(traversabilityClasses.size() <= num)
        traversabilityClasses.resize(num + 1);

    traversabilityClasses[num] = klass;
}

const TraversabilityClass& TraversabilityGrid::getTraversabilityClass(uint8_t klass) const
{
    if(traversabilityClasses.size() <= klass)
    {
        throw std::runtime_error("TraversabilityGrid::Tried to access non existing TraversabilityClass " + boost::lexical_cast< std::string>((int) klass));
    }

    return traversabilityClasses[klass];
}

void TraversabilityGrid::setProbabilityArray()
{
    if(!probabilityArray)
    {
        probabilityArray = &(getData<ArrayType>(PROBABILITY));
        //hm, why is the resize done here ?
        probabilityArray->resize(boost::extents[getCellSizeY()][getCellSizeX()]);
    }
}

void TraversabilityGrid::setTraversabilityArray()
{
    if(!traversabilityArray)
    {
        traversabilityArray = &(getData<ArrayType>(TRAVERSABILITY));
		traversabilityArray->resize(boost::extents[getCellSizeY()][getCellSizeX()]);
    }
}


void TraversabilityGrid::setProbabilityArray() const
{
    TraversabilityGrid *nonConst = const_cast<TraversabilityGrid *>(this);
    nonConst->setProbabilityArray();
}

void TraversabilityGrid::setTraversabilityArray() const
{
    TraversabilityGrid *nonConst = const_cast<TraversabilityGrid *>(this);
    nonConst->setTraversabilityArray();
}

void TraversabilityGrid::setProbability(double probability, size_t x, size_t y)
{
    setProbabilityArray();

    const uint8_t probVal = std::min<uint32_t>(std::numeric_limits< uint8_t >::max(), probability * std::numeric_limits< uint8_t >::max());

    (*probabilityArray)[y][x] = probVal;
}

double TraversabilityGrid::getProbability(size_t x, size_t y) const
{
    const_cast<TraversabilityGrid *>(this)->setProbabilityArray();

    return ((double) (*probabilityArray)[y][x]) / std::numeric_limits< uint8_t >::max();
}

void TraversabilityGrid::serialize(Serialization& so)
{
    so.write<size_t>(std::string("drivabilityClassCount"), traversabilityClasses.size());
    for(size_t i = 0; i < traversabilityClasses.size(); i++)
    {
        so.write(std::string("drivability") + boost::lexical_cast< std::string>((int) i), traversabilityClasses[i].getDrivability());
    }
    envire::Grid< uint8_t >::serialize(so);
}

void TraversabilityGrid::unserialize(Serialization& so)
{
    envire::Grid< uint8_t >::unserialize(so);
    size_t traversabilityClassCount = 0;
    so.read<size_t>(std::string("drivabilityClassCount"), traversabilityClassCount);
    for(size_t i = 0; i < traversabilityClassCount; i++)
    {
        double drivability;
        if(so.read(std::string("drivability") + boost::lexical_cast< std::string>((int) i), drivability))
            setTraversabilityClass(i, TraversabilityClass(drivability));
    }
}

TraversabilityGrid& TraversabilityGrid::operator=(const TraversabilityGrid& other)
{
    const Grid<uint8_t> *obaseGrid = dynamic_cast<const Grid<uint8_t> *>(&other);
    Grid<uint8_t> *baseGrid = dynamic_cast<Grid<uint8_t> *>(this);
    assert(baseGrid);
    assert(obaseGrid);
    *baseGrid = *obaseGrid;

    traversabilityClasses = other.traversabilityClasses;

    traversabilityArray = NULL;
    probabilityArray = NULL;
    return *this;
}


