#include "GridBase.hpp"

using namespace envire;

const std::string GridBase::className = "envire::GridBase";

GridBase::GridBase()
    : width(0), height(0), scalex(0), scaley(0) {}

GridBase::GridBase(size_t width, size_t height, double scalex, double scaley) :
    width(width), height(height), scalex(scalex), scaley(scaley)
{
}

GridBase::~GridBase()
{
}

GridBase::GridBase(Serialization& so)
    : Map<2>( so ), width(0), height(0), scalex(0), scaley(0)
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

bool GridBase::toGrid( Eigen::Vector3d const& point, size_t& m, size_t& n, FrameNode const* frame) const
{
    if (frame)
    {
        Eigen::Vector3d transformed = toMap(point, *frame);
        return toGrid(transformed.x(), transformed.y(), m, n);
    }
    else
    {
        return toGrid(point.x(), point.y(), m, n);
    }
}

bool GridBase::toGrid( double x, double y, size_t& m, size_t& n) const
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

Eigen::Vector3d GridBase::fromGrid(size_t m, size_t n, FrameNode const* frame) const
{
    double map_x, map_y;
    fromGrid(m, n, map_x, map_y);
    if (frame)
    {
        return fromMap(Eigen::Vector3d(map_x, map_y, 0), *frame);
    }
    else
    {
        return Eigen::Vector3d(map_x, map_y, 0);
    }
}

void GridBase::fromGrid( size_t m, size_t n, double& x, double& y) const
{
    x = (m+0.5) * scalex;
    y = (n+0.5) * scaley;
}

bool GridBase::toGrid( const Point2D& point, Position& pos) const
{
    return toGrid( point.x(), point.y(), pos.m, pos.n);
}

void GridBase::fromGrid( const Position& pos, Point2D& point) const
{
    fromGrid( pos.m, pos.n, point.x(), point.y());
}

GridBase::Point2D GridBase::fromGrid( const Position& pos) const
{
    Point2D point;
    fromGrid( pos.m, pos.n, point.x(), point.y());
    return point;
}

bool GridBase::contains( const Position& pos ) const
{
    return (pos.m >= 0 && pos.m < width 
	    && pos.n >= 0 && pos.n < height);
}
        
GridBase::Extents GridBase::getExtents() const
{
    // TODO provide proper extents
    return Extents( Eigen::Vector2d( width * scalex, height * scaley ) ); 
}
