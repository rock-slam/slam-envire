#include "TraversabilityGrid.hpp"
#include <boost/bind.hpp>

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

void addVal(size_t x, size_t y, std::vector<uint8_t> &stats, const TraversabilityGrid::ArrayType &gridData)
{
    stats[gridData[y][x]]++;
}

void TraversabilityGrid::computeStatistic(base::Pose2D pose, double width, double height, double borderWidth)
{
    ArrayType gridData = getGridData(TRAVERSABILITY);
    std::vector<uint8_t> innerStat;
    innerStat.resize(std::numeric_limits<uint8_t>::max());
 
    forEachInRectangle(pose, width, height, boost::bind(addVal, _1, _2, boost::ref(innerStat), boost::ref( gridData)));
}

