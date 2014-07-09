#ifndef POLYGONMAP_HPP
#define POLYGONMAP_HPP

#include <envire/Core.hpp>
#include <stdint.h>

namespace envire
{  

//pimpl class    
class PolygonData;
//pimpl class
class PolygonSetData;

class PolygonSet;
    
class Polygon
{
    friend class PolygonSet;
    friend class PolygonSetData;
public:
    Polygon(const std::vector<Eigen::Vector2d> &points);
    ~Polygon();
    void move(const base::Pose &pose);
    
    /**
     * Everything inside this area will be used
     * for closest distance computation. Note that
     * the whole area must be outside of the polygon.
     * 
     * If not set, only intersection will be computed
     * */
    void setDistanceCalculationArea(const std::vector<Eigen::Vector2d> &points);
    
private:
    PolygonData *data;
};


class PolygonSet
{
public:
    PolygonSet();
    ~PolygonSet();
    void add(Polygon &poly);
    bool isIntersecting(const Polygon &poly);
    bool isInside(const Polygon &poly);
    double getMinimalDistance(const envire::Polygon& poly);
private:
    void doQuery(const Polygon &poly);
    PolygonSetData *data;
};
    
class PolygonMap : public Map<2>
{
    ENVIRONMENT_ITEM( PolygonMap )
public:
    PolygonMap();
    
    virtual Extents getExtents() const;
private:
    PolygonSet drivableArea;
    PolygonSet obstacles;
    PolygonSet parkingSpots;
    
};

}
#endif // POLYGONMAP_HPP
