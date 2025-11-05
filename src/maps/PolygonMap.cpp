#include "PolygonMap.hpp"
#include <envire/core/Serialization.hpp>

#ifdef BOX2D_2_4
#include <box2d/b2_collision.h>
#include <box2d/b2_distance.h>
#include <box2d/box2d.h>
#else
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Box2D.h>
#endif

namespace envire
{  
ENVIRONMENT_ITEM_DEF( PolygonMap )
    

class PolygonData
{
public:
    b2PolygonShape shape;
    b2PolygonShape distanceCompShape;
    b2Transform pose;
    b2AABB aabb;
    int proxyId;
    mutable bool moved;
    
    void setShape(b2PolygonShape &shape, const std::vector<Eigen::Vector2d>& points) const
    {
	std::vector<b2Vec2> bp;
	bp.resize(points.size());
	std::vector<b2Vec2>::iterator bi = bp.begin();
	
	for(std::vector<Eigen::Vector2d>::const_iterator it = points.begin(); it != points.end();it++)
	{
	    bi->Set(it->x(), it->y());
	    bi++;
	}

	shape.Set(bp.data(), bp.size());
    }
};

Polygon::Polygon(const std::vector<Eigen::Vector2d>& points) : data(new PolygonData)
{
    data->setShape(data->shape, points);
    data->pose.SetIdentity();
    data->moved = true;
    setDistanceCalculationArea(points);
}

void Polygon::setDistanceCalculationArea(const std::vector<Eigen::Vector2d>& points)
{
    data->setShape(data->distanceCompShape, points);
    data->distanceCompShape.ComputeAABB(&(data->aabb), data->pose, 0);
}

Polygon::~Polygon()
{
    delete data;
}


void Polygon::move(const base::Pose& pose)
{
    data->moved = true;
    b2Vec2 pos(pose.position.x(), pose.position.y());
//     std::cout << "Moved to " << pos.x << " " << pos.y << std::endl;
    data->pose.Set(pos, pose.getYaw());
    data->distanceCompShape.ComputeAABB(&(data->aabb), data->pose, 0);
}




class PolygonSetData
{
public:
    PolygonSetData() : curTestPoly(NULL), collisionPoly(NULL), hasCollision(false)
    {
	minDistance.distance = std::numeric_limits< float >::max();
    }
    std::vector<envire::Polygon *> polygons;
    b2BroadPhase broadPhase;
    bool QueryCallback(int32_t proxyId);
    
    //collision test data
    const envire::Polygon *curTestPoly;
    const envire::Polygon *collisionPoly;
    bool hasCollision;
    b2DistanceOutput minDistance;
};

PolygonMap::PolygonMap()
{
    
}

PolygonSet::PolygonSet() : data(new PolygonSetData())
{

}

PolygonSet::~PolygonSet()
{
    delete data;
}

void PolygonSet::add(envire::Polygon& poly)
{
    int proxyId = data->broadPhase.CreateProxy(poly.data->aabb, NULL);
    if(data->polygons.size() <= proxyId)
	data->polygons.resize(proxyId + 1, NULL);
	
    poly.data->proxyId = proxyId;

    data->polygons[proxyId] = &poly;
}

Map< 2 >::Extents PolygonMap::getExtents() const
{
    return Extents(0,0);
}

void PolygonSet::doQuery(const envire::Polygon& poly)
{
    //check if we allready computed distance and intersection
    if((&poly == data->curTestPoly) && !poly.data->moved)
	return;
    
//     std::cout << "Performing querry" << std::endl;
    
    poly.data->moved = false;
    data->hasCollision = false;
    data->minDistance.distance = std::numeric_limits< float >::max();
    data->curTestPoly = &poly;    
    data->broadPhase.Query(data, poly.data->aabb);
}

double PolygonSet::getMinimalDistance(const envire::Polygon& poly)
{
    doQuery(poly);
    
    return data->minDistance.distance;
}

bool PolygonSet::isInside(const envire::Polygon& poly)
{
    doQuery(poly);

    if(!data->hasCollision)
	return false;
    
    const b2PolygonShape *shape1 = &(poly.data->shape);
    const b2PolygonShape *shape2 = &(data->collisionPoly->data->shape);
    
    //perform point test for every vertex in poly
#ifdef BOX2D_2_4
    for(int i = 0;i < shape1->m_count; i++)
#else
    for(int i = 0;i < shape1->GetVertexCount(); i++)
#endif
    {
	if(!shape2->TestPoint(poly.data->pose, shape1->m_vertices[i]))
	    return false;
    }
    return true;
}

bool PolygonSet::isIntersecting(const envire::Polygon& poly)
{
    doQuery(poly);

    return data->hasCollision;
}

/**
 * This function gets called if we hit a leaf in the AABB-Tree.
 * In this case we compute the distance of the two shapes
 * inside the colliding AABBs. If the distance is really small
 * we assume a collision happended and terminate the tree query.
 * */
bool PolygonSetData::QueryCallback(int32_t proxyId)
{
//     std::cout << "Querry callback proxyId " << proxyId << " test id " << curTestPoly->data->proxyId << std::endl; 
    b2DistanceInput input;
    input.proxyA.Set(&(curTestPoly->data->shape), 0);
    input.proxyB.Set(&(polygons[proxyId]->data->shape), 0);
    input.transformA = curTestPoly->data->pose;
//     std::cout << "Pos A " << input.transformA.p.x << " " << input.transformA.p.y << std::endl;
    input.transformB = polygons[proxyId]->data->pose;
//     std::cout << "Pos B " << input.transformB.p.x << " " << input.transformB.p.y << std::endl;
    input.useRadii = true;

    b2SimplexCache cache;
    cache.count = 0;

    //compute distance of shapes
    b2DistanceOutput output;    
    b2Distance(&output, &cache, &input);

    //this is stollen from the box2d code
    bool collision = output.distance < 10.0f * b2_epsilon;
    if(collision)
    {
	hasCollision = true;
	collisionPoly = polygons[proxyId];
    }
    
    std::cout << output.pointA.x << " " << output.pointA.y << std::endl;
    std::cout << output.pointB.x << " " << output.pointB.y << std::endl;
    
    //save minimum distance
    if(output.distance < minDistance.distance)
	minDistance = output;
    
    //stop querry if we have an collision
    return !collision;
}


}