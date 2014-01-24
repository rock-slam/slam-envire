#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "EnvireWidget.hpp"
#include "envire/maps/Grids.hpp"

using namespace envire;

BOOST_AUTO_TEST_CASE( grid_test ) 
{
    // set up test environment
    QtThreadedWidget<envire::EnvireWidget> app;
    app.start();
    Environment* env = app.getWidget()->getEnvironment();

    envire::Grid<float> *grid = new envire::Grid<float>( 50, 50, 0.1, 0.1, 0, 0 );

    grid->getGridData()[20][10] = 5.0;
    grid->getGridData()[30][20] = 10.0;

    env->attachItem( grid );
    env->setFrameNode( grid, env->getRootNode() );

    usleep(1000*1000*10);
}

