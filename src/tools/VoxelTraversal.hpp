#ifndef ENVIRE_TOOLS_VOXELTRAVERSAL_HPP__
#define ENVIRE_TOOLS_VOXELTRAVERSAL_HPP__

namespace envire
{


/**
 * starting from the current cell, we want to advance towards the
 * light source until either the cell of the light source is
 * reached, or we leave the grid.
 *
 * the algorithm is based on 
 * "A Fast Voxel Traversal Algorithm for Ray Tracing"
 * John Amanatides, Andrew Woo
 * http://www.cse.yorku.ca/~amana/research/grid.pdf
 *
 */
class VoxelTraversal
{
protected:
    int cx, cy;
    int stepx, stepy;
    double deltax, deltay;
    double maxx, maxy;

    const GridBase& grid;
    bool valid;

public:
    explicit VoxelTraversal( const GridBase& grid )
	: grid( grid ), valid( false )
    {}

    void init( const Eigen::Vector3d& orig3, const Eigen::Vector3d& dir3 )
    {
	assert( dir3.norm() > .0 );

	Eigen::Vector2d
	    orig = orig3.head<2>(),
	    dir = dir3.head<2>();

	envire::GridBase::Position pos;
	double xmod, ymod;
	valid = grid.toGrid( orig.x(), orig.y(), pos.x, pos.y, xmod, ymod );
	cx = pos.x;
	cy = pos.y;

	stepx = dir.x() > 0 ? 1 : -1;
	stepy = dir.y() > 0 ? 1 : -1;

	// this is the distance along the ray until a new cell is reached
	deltax = (dir / dir.x() * grid.getScaleX()).norm();
	deltay = (dir / dir.y() * grid.getScaleY()).norm();

	// starting distance until a new cell is reached.
	maxx = deltax * xmod;
	maxy = deltay * ymod; 
    }

    Eigen::Vector3d getPosition()
    {
	return Eigen::Vector3d( maxx / deltax, maxy / deltay, .0 );
    }

    bool step( GridBase::Position &pos )
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
	if( !(cx >= 0 && cx < (int)grid.getCellSizeX() && cy >= 0 && cy < (int)grid.getCellSizeY()) )
	    valid = false;

	pos = GridBase::Position( cx, cy );
	return valid;
    }
};
}
#endif
