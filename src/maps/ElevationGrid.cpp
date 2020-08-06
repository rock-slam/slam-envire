#include "ElevationGrid.hpp"

using namespace envire;

ENVIRONMENT_ITEM_DEF( ElevationGrid )
const std::string ElevationGrid::ELEVATION = "elevation_max"; // this will reference the max band
const std::string ElevationGrid::ELEVATION_MIN = "elevation_min";
const std::string ElevationGrid::ELEVATION_MAX = "elevation_max";
const std::string ElevationGrid::ILLUMINATION = "illumination";
const std::string ElevationGrid::VISIBILITY = "visibility";
const std::vector<std::string> ElevationGrid::bands = { ElevationGrid::ELEVATION_MIN,
    ElevationGrid::ELEVATION_MAX, ElevationGrid::ILLUMINATION, ElevationGrid::VISIBILITY };

Eigen::Vector3d envire::ElevationGrid::getNormal(const envire::GridBase::Position& pos) const
{
    size_t m = pos.x, n = pos.y;

    const ArrayType &grid( getGridData( ELEVATION ) );
    double slope_x = (grid[n][m-1] - grid[n][m+1]) / (getScaleX()*2.0);
    double slope_y = (grid[n-1][m] - grid[n+1][m]) / (getScaleY()*2.0);

    return Eigen::Vector3d( slope_x, slope_y, 1.0 ).normalized();
}

Eigen::Vector3d envire::ElevationGrid::getNormal(const Point2D& pos) const
{
    size_t m, n;
    if (!toGrid(pos.x(), pos.y(), m, n ))
        throw std::runtime_error("provided coordinates are out of the grid");

    return getNormal( Position( m, n ) );
}

double envire::ElevationGrid::getElevation(const Point2D& pos) const
{
    size_t m, n;
    if (!toGrid(pos.x(), pos.y(), m, n ))
        throw std::runtime_error("provided coordinates are out of the grid");

    double x, y;
    fromGrid( m, n, x, y );

    //X and Y in the cell
    x -= pos.x();
    y -= pos.y();

    Eigen::Vector3d normal = getNormal( pos );
    double slope_x = normal.x() / normal.z();
    double slope_y = normal.y() / normal.z();

    double height =
        getFromRaster( ELEVATION, m, n ) +
        x * slope_x +
        y * slope_y;

    return height;
}
