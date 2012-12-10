#include "GridIllumination.hpp"
#include <envire/maps/Grids.hpp>

using namespace envire;
using namespace Eigen;

ENVIRONMENT_ITEM_DEF( GridIllumination )

GridIllumination::GridIllumination()
    : lightSource( base::Vector3d::Zero() ), lightDiameter( 0.0 ), band( ElevationGrid::ILLUMINATION )
{
}

bool GridIllumination::updateAll()
{
    // get output grid
    ElevationGrid* grid = getOutput<envire::ElevationGrid*>();

    // and get the array
    ElevationGrid::ArrayType &harray = grid->getGridData( ElevationGrid::ELEVATION_MAX );
    ElevationGrid::ArrayType &iarray = grid->getGridData( band );

    // get the position of the light source
    ElevationGrid::Position lightPos;
    bool lightInGrid = grid->toGrid( lightSource, lightPos.x, lightPos.y );

    for( size_t x = 0; x < grid->getCellSizeX(); x++ )
    {
	for( size_t y = 0; y < grid->getCellSizeY(); y++ )
	{
	    Vector3d cell = grid->fromGrid( x, y );
	    // get z-value from array
	    cell.z() = harray[y][x];
	    Vector3d dir3 = lightSource - cell;
	    // the direction to the light source in 2d
	    Vector2d dir = dir3.head<2>();

	    // min/max height at 1m distance where the light is visible
	    double 
		lightMin = (dir3.z() - .5*lightDiameter) / dir.norm(),
		lightMax = (dir3.z() + .5*lightDiameter) / dir.norm();

	    // starting from the current cell, we want to advance towards the
	    // light source until either the cell of the light source is
	    // reached, or we leave the grid.
	    //
	    // the algorithm is based on 
	    // "A Fast Voxel Traversal Algorithm for Ray Tracing"
	    // John Amanatides, Andrew Woo
	    // http://www.cse.yorku.ca/~amana/research/grid.pdf
	    //
	    size_t cx = x, cy = y;
	    // set directions in x and y
	    int 
		stepx = dir.x() > 0 ? 1 : -1,
		stepy = dir.y() > 0 ? 1 : -1;
	    // this is the distance along the ray until a new cell is reached
	    const double 
		deltax = (dir / dir.x() * grid->getScaleX()).norm(),
		deltay = (dir / dir.y() * grid->getScaleY()).norm();
	    // starting distance until a new cell is reached.
	    // since we start in the center of the cell, this is half the 
	    // deltax, and deltay
	    double 
		maxx = deltax * 0.5,
		maxy = deltay * 0.5; 

	    double maxLight = 0.0; 
	    while( true )
	    {
		// advance to next cell
		if( maxx < maxy )
		{
		    maxx += deltax;
		    cx += stepx;
		}
		else
		{
		    maxy += deltay;
		    cy += stepy;
		}

		// see if we are still within bounds
		if( !(cx >= 0 && cx < grid->getCellSizeX() && cy >= 0 && cy < grid->getCellSizeY()) )
		    break;
		// check if we already are on the light-source
		if( lightInGrid && ElevationGrid::Position(cx, cy) == lightPos )
		    break;

		// get distance value on x/y plane
		double dist = (grid->fromGrid( cx, cy ).head<2>() - cell.head<2>()).norm();

		// now get the elevationvalue from the grid relative to the
		// current cell
		double zDiff = harray[cy][cx] - cell.z();
		// z height normalized to dist and mapped to min/max light
		double zRel = (zDiff / dist - lightMin ) / (lightMax - lightMin); 
		maxLight = std::max( maxLight, zRel );
	    }

	    // set the light value in the illumination band
	    iarray[y][x] = 1.0 - std::min( maxLight, 1.0 );
	}
    }

    return true;
}

void GridIllumination::setLightSource( const base::Vector3d& ls, double diameter )
{
    lightSource = ls;
    lightDiameter = diameter;
}

void GridIllumination::setOutputBand( const std::string& band )
{
    this->band = band;
}
