#include "TraversabilityGrid.hpp"
#include <boost/bind.hpp>
#include <tools/RadialLookUpTable.hpp>
#include <tools/BoxLookUpTable.hpp>
#include <Eigen/Geometry>

using namespace envire;
using namespace Eigen;

ENVIRONMENT_ITEM_DEF( TraversabilityGrid )
const std::string TraversabilityGrid::TRAVERSABILITY = "traversability";
static const std::vector<std::string> &initTraversabilityBands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(TraversabilityGrid::TRAVERSABILITY);
  }
  return bands;
}
const std::vector<std::string> &TraversabilityGrid::bands = initTraversabilityBands();


class StatisticHelper
{
    static RadialLookUpTable *lut;
    static BoxLookUpTable *boxLut;
    const base::Pose2D &pose;
    Eigen::Rotation2D<double> inverseOrientation;
    const TraversabilityGrid::ArrayType &gridData;
    TraversabilityStatistic *innerStats;
    TraversabilityStatistic *outerStats;
    double scaleX;
    double scaleY;
    int xCenter;
    int yCenter;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StatisticHelper(const base::Pose2D &pose, const TraversabilityGrid &grid, double width, double height, double borderWidth) : pose(pose), 
    inverseOrientation(Eigen::Rotation2D<double>(pose.orientation).inverse()), gridData(grid.getGridData(TraversabilityGrid::TRAVERSABILITY))
    , scaleX(grid.getScaleX()), scaleY(grid.getScaleY())
    {
        if(!lut)
            lut = new RadialLookUpTable();
        lut->recompute(grid.getScaleX(), std::max(width, height));
        
        if(!boxLut)
            boxLut = new BoxLookUpTable();
        //note the scale should be higher than the grid scale because 
        //of the roation. Else we get aliasing problems.
        boxLut->recompute(scaleX / 10.0, width, height, borderWidth);
        
        xCenter = pose.position.x() / grid.getScaleX();
        yCenter = pose.position.y() / grid.getScaleY();
    }

    void addInnerVal(size_t x, size_t y)
    {
        innerStats->addMeasurement(gridData[y][x], lut->getDistance(x-xCenter, y-yCenter));
    }
    
    void addOuterVal(size_t x, size_t y)
    {
        Vector2d posAligned = inverseOrientation * (Vector2d(x * scaleX, y * scaleY) - pose.position);
        
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

void TraversabilityGrid::computeStatistic(const base::Pose2D &pose, double width, double height, TraversabilityStatistic &innerStatistic) const
{
    StatisticHelper helper(pose, *this, width, height, 0);
    helper.setInnerStatistic(&innerStatistic);
    forEachInRectangle(pose, width, height, boost::bind( &StatisticHelper::addInnerVal, &helper,  _1, _2));
}

void TraversabilityGrid::computeStatistic(const base::Pose2D &pose, double width, double height, double borderWidth, TraversabilityStatistic &innerStatistic, TraversabilityStatistic &outerStatistic) const
{
    StatisticHelper helper(pose, *this, width, height, borderWidth);
    helper.setInnerStatistic(&innerStatistic);
    helper.setOuterStatistic(&outerStatistic);
    forEachInRectangles(pose, width, height, boost::bind( &StatisticHelper::addInnerVal, &helper,  _1, _2), 
                        width + borderWidth, height + borderWidth, boost::bind( &StatisticHelper::addOuterVal, &helper,  _1, _2));
}

const TraversabilityClass& TraversabilityGrid::getWorstTraversabilityClassInRectangle(const base::Pose2D& pose, double width, double height) const
{
    double curDrivability = std::numeric_limits< double >::max();
    int curClass = -1;
    
    TraversabilityStatistic innerStatistic;
    computeStatistic(pose, width, height, innerStatistic);

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
        std::cout << "Pose " << pose.position.transpose() << " width " << width << " height " << height << " Total Count " << innerStatistic.getTotalCount() << std::endl;
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

void TraversabilityGrid::setTraversabilityClass(uint8_t num, const TraversabilityClass& klass)
{
    if(traversabilityClasses.size() >= num)
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

void TraversabilityGrid::serialize(Serialization& so)
{
    so.write<size_t>(std::string("drivabilityClassCount"), traversabilityClasses.size());
    for(uint8_t i = 0; i < traversabilityClasses.size(); i++)
    {
        so.write(std::string("drivability") + boost::lexical_cast< std::string>((int) i), traversabilityClasses[i].getDrivability());
    }
    envire::Grid< uint8_t >::serialize(so);
}

void TraversabilityGrid::unserialize(Serialization& so)
{
    size_t traversabilityClassCount = 0; 
    so.read<size_t>(std::string("drivabilityClassCount"), traversabilityClassCount); 
    for(uint8_t i = 0; i < traversabilityClassCount; i++)
    {
        double drivability;
        if(so.read(std::string("drivability") + boost::lexical_cast< std::string>((int) i), drivability))
            setTraversabilityClass(i, TraversabilityClass(drivability));
    }
    envire::Grid< uint8_t >::unserialize(so);
}

