#include "GridBase.hpp"

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
    : Map<2>( so )
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

bool GridBase::toGrid( double x, double y, size_t& m, size_t& n ) const
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

void GridBase::fromGrid( size_t m, size_t n, double& x, double& y ) const
{
    x = (m+0.5) * scalex;
    y = (n+0.5) * scaley;
}

bool GridBase::toGrid( const Point2D& point, Position& pos ) const
{
    return toGrid( point.x(), point.y(), pos.m, pos.n );
}

void GridBase::fromGrid( const Position& pos, Point2D& point ) const
{
    fromGrid( pos.m, pos.n, point.x(), point.y() );
}

GridBase::Point2D GridBase::fromGrid( const Position& pos ) const
{
    Point2D point;
    fromGrid( pos.m, pos.n, point.x(), point.y() );
    return point;
}
        
GridBase::Extents GridBase::getExtents() const
{
    // TODO provide proper extents
    return Extents( Eigen::Vector2d( width * scalex, height * scaley ) ); 
}
