#define BOOST_TEST_MODULE OCCUPANCYGRIDTest
#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <cmath>
#include "envire/Core.hpp"

#include <envire/maps/OccupancyGrid.hpp>

using namespace envire;

BOOST_AUTO_TEST_CASE( BaseOccupancyGridTest ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    OccupancyGrid *map = new OccupancyGrid(100,100, 0.1, 0.1);
    env->attachItem( map );
    
    GridBase::Point2D point = map->getVehicleCellPosition();
    BOOST_CHECK_CLOSE(point.x(),50,1e-6);
    BOOST_CHECK_CLOSE(point.y(),50,1e-6);
}


BOOST_AUTO_TEST_CASE( OccupancyGrid_NORMALIZEEGO ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    OccupancyGrid *map = new OccupancyGrid(200,200, 1, 1);
    env->attachItem( map );
    
    map->updateProbability(2,2,1.0);
    BOOST_CHECK_CLOSE(map->getProbability(2,2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(102,102), 1.0, 1e-3);

    map->updateProbability(4,2,1.0);
    BOOST_CHECK_CLOSE(map->getProbability(4,2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(104,102), 1.0, 1e-3);

    //move vehicle from the center to the circle of a radius of one
    //all cell values are moved accordingly
    map->normalizeEgoGrid(1);
    BOOST_CHECK_CLOSE(map->getProbability(2,2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getProbability(4,2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(101,102), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(103,102), 1.0, 1e-3);
    GridBase::Point2D point = map->getVehicleCellPosition();
    BOOST_CHECK_CLOSE(point.x(),99,1e-6);
    BOOST_CHECK_CLOSE(point.y(),100,1e-6);

    //turning around 180° this means the grid will be moved by 2 meter
    //to stay on the circle and pointing to the center
    //the values are now behind the vehicle
    map->updateVehicleOrientation(M_PI);
    map->normalizeEgoGrid(1);
    BOOST_CHECK_CLOSE(map->getProbability(-2,-2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getProbability(-4,-2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(103,102), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(105,102), 1.0, 1e-3);
    point = map->getVehicleCellPosition();
    BOOST_CHECK_CLOSE(point.x(),101,1e-6);
    BOOST_CHECK_CLOSE(point.y(),100,1e-6);

    for(int j = 95; j<105;++j)
    {
        for(int i = 90;i<110;++i)
            std::cout << map->getCellProbability(i,j) << " ";
        std::cout << std::endl;
    }

    // turn 90°
    map->updateVehicleOrientation(M_PI*0.5);
    map->normalizeEgoGrid(1);
    BOOST_CHECK_CLOSE(map->getProbability(-2,2), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getProbability(-2,4), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(102,101), 1.0, 1e-3);
    BOOST_CHECK_CLOSE(map->getCellProbability(104,101), 1.0, 1e-3);
    point = map->getVehicleCellPosition();
    BOOST_CHECK_CLOSE(point.x(),100,1e-6);
    BOOST_CHECK_CLOSE(point.y(),99,1e-6);

    //set vehicle position and normailize so that the vehicle will
    //be moved back to the center
    map->updateVehicleOrientation(0);
    map->updateVehicleCellPosition(100,100);
    map->updateVehiclePosition(10.5,10.5);
    map->normalizeEgoGrid(0);
    point = map->getVehicleCellPosition();
    BOOST_CHECK_EQUAL(point.x(),100.5);
    BOOST_CHECK_EQUAL(point.y(),100.5);

    //check if old values got deleted

    //env->serialize("/home/aduda/dev/environment");
}
