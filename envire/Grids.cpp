#include "Grids.hpp"

using namespace envire;

const std::string TraversabilityGrid::className = "envire::TraversabilityGrid";
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


const std::string ConfidenceGrid::className = "envire::ConfidenceGrid";
const std::string ConfidenceGrid::CONFIDENCE = "confidence";
static const std::vector<std::string> &initConfidenceBands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(ConfidenceGrid::CONFIDENCE);
  }
  return bands;
}
const std::vector<std::string> &ConfidenceGrid::bands = initConfidenceBands();


const std::string ElevationGrid::className = "envire::ElevationGrid";
const std::string ElevationGrid::ELEVATION = "elevation_max"; // this will reference the max band
const std::string ElevationGrid::ELEVATION_MIN = "elevation_min";
const std::string ElevationGrid::ELEVATION_MAX = "elevation_max";
static const std::vector<std::string> &initElevationBands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(ElevationGrid::ELEVATION_MIN);
    bands.push_back(ElevationGrid::ELEVATION_MAX);
  }
  return bands;
}
const std::vector<std::string> &ElevationGrid::bands = initElevationBands();


const std::string OccupancyGrid::className = "envire::OccupancyGrid";
const std::string OccupancyGrid::OCCUPANCY = "occupancy";
static const std::vector<std::string> &initOccupancyBands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(OccupancyGrid::OCCUPANCY);
  }
  return bands;
}
const std::vector<std::string> &OccupancyGrid::bands = initOccupancyBands();


const std::string ImageRGB24::className = "envire::ImageRGB24";
const std::string ImageRGB24::R = "r";
const std::string ImageRGB24::G = "g";
const std::string ImageRGB24::B = "b";
static const std::vector<std::string> &initImageRGB24Bands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(ImageRGB24::R);
    bands.push_back(ImageRGB24::G);
    bands.push_back(ImageRGB24::B);
  }
  return bands;
}
const std::vector<std::string> &ImageRGB24::bands = initImageRGB24Bands();

