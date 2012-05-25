#define BOOST_TEST_MODULE MLSTest 
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
    int i=0;
    boost::scoped_ptr<Environment> env( new Environment() );
    OccupancyGrid *map = new OccupancyGrid(100,100, 0.1, 0.1);
    
    env->attachItem( map );
    
    size_t center_x, center_y;
    map->toGrid(0,0,center_x,center_y);
    
    BOOST_CHECK_EQUAL(map->getProbability(center_x,center_y), 0.5);
    map->updateProbability(0,0,0.5);
    BOOST_CHECK_EQUAL(map->getProbability(center_x,center_y), 0.5);
   
    map->clear(0.5);
    BOOST_CHECK_EQUAL(map->getProbability(center_x,center_y), 0.5);

    map->updateProbability(0,0,0.6);
    BOOST_CHECK_CLOSE(map->getProbability(center_x,center_y), 0.6,1e-5);
    
    map->clear(0.5);
    map->updateProbability(0,0,0.4);
    BOOST_CHECK_CLOSE(map->getProbability(center_x,center_y), 0.4,1e-5);
    
    map->clear(0.5);
    map->updateProbability(0,0,1-1e-6);
    BOOST_CHECK_CLOSE(map->getProbability(center_x,center_y), 1.0, 1e-3);

    map->updateProbability(0,0,1.0);
    BOOST_CHECK( std::isnan( map->getProbability(center_x,center_y) )); 
    
    map->clear(0.5);
    map->updateProbability(0,0,0.0);
    BOOST_CHECK_EQUAL(map->getProbability(center_x,center_y), 0.0);
    
    BOOST_CHECK( std::isnan( map->getProbability(500,500) )); 

}

