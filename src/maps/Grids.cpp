#include "Grids.hpp"

using namespace envire;

static envire::SerializationPlugin< Grid<double> >  Grid_double_plugin("Grid_d");
static envire::SerializationPlugin< Grid<float> >   Grid_float_plugin("Grid_f");
static envire::SerializationPlugin< Grid<int> >     Grid_int_plugin("Grid_i");
static envire::SerializationPlugin< Grid<uint8_t> > Grid_byte_plugin("Grid_h");

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


ENVIRONMENT_ITEM_DEF( ConfidenceGrid )
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

ENVIRONMENT_ITEM_DEF( ElevationGrid )
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


ENVIRONMENT_ITEM_DEF( OccupancyGrid )
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

ENVIRONMENT_ITEM_DEF( ImageRGB24 )
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

