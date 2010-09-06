#include "Grid.hpp"

using namespace envire;

const std::string GridBase::className = "envire::GridBase";

GridBase::GridBase(size_t width, size_t height, double scalex, double scaley) :
    width(width), height(height), scalex(scalex), scaley(scaley)
{
}

GridBase::~GridBase()
{
}

GridBase::GridBase(Serialization& so)
    : CartesianMap( so )
{
    unserialize(so);
}

void GridBase::serialize(Serialization& so)
{
    CartesianMap::serialize(so);

    so.setClassName( className );
    so.write("width", width );
    so.write("height", height );
    so.write("scalex", scalex );
    so.write("scaley", scaley );
}

void GridBase::unserialize(Serialization& so)
{
    so.setClassName(className);
    so.read("width", width );
    so.read("height", height );
    so.read("scalex", scalex );
    so.read("scaley", scaley );

}

/** @return true if the given coordinates x and y are on the grid
 * array indices are returned in @param m and @param n
 */
bool GridBase::toGrid( double x, double y, size_t& m, size_t& n )
{
    size_t am = floor(x/scalex);
    size_t an = floor(y/scaley);
    if( 0 <= am && am < width && 0 <= an && an < height )
    {
	m = am;
	n = an;
	return true;
    }
    else {
	return false;
    }
}
        
