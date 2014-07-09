#define BOOST_TEST_MODULE PolyMapTests 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include <envire/maps/PolygonMap.hpp>

using namespace envire;
using namespace Eigen;

BOOST_AUTO_TEST_CASE( test_nonIntersecting ) 
{
    std::vector<Eigen::Vector2d> poly1Points;
    poly1Points.push_back(Vector2d(0,0));
    poly1Points.push_back(Vector2d(1,0));
    poly1Points.push_back(Vector2d(1,1));
    poly1Points.push_back(Vector2d(0,1));
    Polygon poly1(poly1Points);
    
    std::vector<Eigen::Vector2d> poly1OutPoints;
    poly1OutPoints.push_back(Vector2d(-5,-5));
    poly1OutPoints.push_back(Vector2d(5,-5));
    poly1OutPoints.push_back(Vector2d(5,5));
    poly1OutPoints.push_back(Vector2d(-5, 5));
    poly1.setDistanceCalculationArea(poly1OutPoints);

    std::vector<Eigen::Vector2d> poly2Points;
    poly2Points.push_back(Vector2d(2,2));
    poly2Points.push_back(Vector2d(3,2));
    poly2Points.push_back(Vector2d(3,3));
    poly2Points.push_back(Vector2d(2,3));
    Polygon poly2(poly2Points);

    PolygonSet set;
    set.add(poly2);

    bool inside = set.isInside(poly1);
    BOOST_CHECK(!inside);

    bool intersection = set.isIntersecting(poly1);
    BOOST_CHECK(!intersection);
    
    double dist = set.getMinimalDistance(poly1);
    std::cout << "dist is " << dist << std::endl;
    BOOST_CHECK_CLOSE( dist, sqrt(2), 2 );
    
}

std::vector<Eigen::Vector2d> getBoxPoly(double edgeLenght)
{
    double size = edgeLenght / 2.0;
    std::vector<Eigen::Vector2d> points;
    points.push_back(Vector2d(-size,-size));
    points.push_back(Vector2d(size,-size));
    points.push_back(Vector2d(size,size));
    points.push_back(Vector2d(-size,size));
    return points;
}

BOOST_AUTO_TEST_CASE( test_nonIntersectingSame ) 
{
    PolygonSet set;

    double edgeSize = 2.0;
    double freeSpace = 1.0;
    
    for(int i = 0; i < 10000; i++)
    {
	std::vector<Eigen::Vector2d> poly1Points = getBoxPoly(edgeSize);
	Polygon *poly1 = new Polygon(poly1Points);
	
	base::Pose pose;
	pose.position.x() = i*(edgeSize + freeSpace);
	
	poly1->move(pose);
	
	set.add(*poly1);
	
    }

    std::vector<Eigen::Vector2d> poly2Points = getBoxPoly(1.0);
    std::vector<Eigen::Vector2d> polyOutPoints = getBoxPoly(3.0);
    Polygon poly2(poly2Points);
    poly2.setDistanceCalculationArea(polyOutPoints);
    
    base::Pose pose;
    pose.position.x() = 80;
    pose.position.y() = 2;
    poly2.move(pose);


    bool inside = set.isInside(poly2);
    BOOST_CHECK(!inside);

    base::Time start = base::Time::now();
    bool intersection = set.isIntersecting(poly2);
    BOOST_CHECK(!intersection);

    base::Time end = base::Time::now();

    double dist = set.getMinimalDistance(poly2);
    std::cout << "dist is " << dist << " Time " << end-start <<  std::endl;
//     BOOST_CHECK_CLOSE( dist, sqrt(2), 2 );
    
}