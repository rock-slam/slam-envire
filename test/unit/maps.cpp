#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include <envire/maps/Grids.hpp>

using namespace envire;

BOOST_AUTO_TEST_CASE( test_elevationgrid ) 
{
    ElevationGrid grid( 3, 3, 0.5, 0.5 );

    grid.get( 0.75, 0.75 ) = 1.0;
    grid.get( 0, 0.75 ) = 0.0;
    grid.get( 1.25, 0.75 ) = 2.0;
    grid.get( 0.75, 0.25 ) = -1.0;
    grid.get( 0.75, 1.25 ) = 3.0;

    BOOST_CHECK( Eigen::Vector3d( -2.0, -4.0, 1.0 ).normalized().isApprox( grid.getNormal( Eigen::Vector2d( 0.75, 0.75 ) ) ) );
    BOOST_CHECK_CLOSE( 1.5, grid.getElevation( Eigen::Vector2d( 1.0 - 1e-9, 0.75 ) ), 1e-5 );

}

