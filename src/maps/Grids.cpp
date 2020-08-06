#include "Grids.hpp"
#include <boost/filesystem/operations.hpp>

using namespace envire;

bool envire::fileExists(std::string const& path)
{
    return boost::filesystem::exists(path);
}

template class Grid<double>;
template class Grid<float>;
template class Grid<uint8_t>;
template class Grid<int16_t>;
template class Grid<uint16_t>;
template class Grid<int32_t>;
template class Grid<uint32_t>;

static envire::SerializationPlugin< Grid<double> >   Grid_double_plugin("Grid_d");
static envire::SerializationPlugin< Grid<float> >    Grid_float_plugin("Grid_f");
static envire::SerializationPlugin< Grid<uint8_t> >  Grid_uint8_plugin("Grid_h");
static envire::SerializationPlugin< Grid<int16_t> >  Grid_int16_plugin("Grid_s");
static envire::SerializationPlugin< Grid<uint16_t> > Grid_uint16_plugin("Grid_t");
static envire::SerializationPlugin< Grid<int32_t> >  Grid_int32_plugin("Grid_i");
static envire::SerializationPlugin< Grid<uint32_t> > Grid_uint32_plugin("Grid_j");

ENVIRONMENT_ITEM_DEF( ConfidenceGrid )
const std::string ConfidenceGrid::CONFIDENCE = "confidence";
const std::vector<std::string> ConfidenceGrid::bands = { ConfidenceGrid::CONFIDENCE };

ENVIRONMENT_ITEM_DEF( DistanceGrid )
const std::string DistanceGrid::DISTANCE = "distance";
const std::string DistanceGrid::CONFIDENCE = "confidence";
const std::vector<std::string> DistanceGrid::bands = { DistanceGrid::DISTANCE, DistanceGrid::CONFIDENCE };

void DistanceGrid::copyFromDistanceImage( const base::samples::DistanceImage& dimage )
{
    envire::DistanceGrid::ArrayType& distance =
	getGridData( envire::DistanceGrid::DISTANCE );

    // copy the content not very performant but should do for now.
    for( size_t y = 0; y<cellSizeY; y++ )
    {
        for( size_t x = 0; x<cellSizeX; x++ )
        {
	    distance[y][x] = dimage.data[y*cellSizeX+x];
	}
    }
}

ENVIRONMENT_ITEM_DEF( OccupancyGrid )
const std::string OccupancyGrid::OCCUPANCY = "occupancy";
const std::vector<std::string> OccupancyGrid::bands = { OccupancyGrid::OCCUPANCY };

ENVIRONMENT_ITEM_DEF( ImageRGB24 )
const std::string ImageRGB24::R = "r";
const std::string ImageRGB24::G = "g";
const std::string ImageRGB24::B = "b";
const std::vector<std::string> ImageRGB24::bands = { ImageRGB24::R, ImageRGB24::G, ImageRGB24::B };

