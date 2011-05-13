#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"
#include "envire/maps/MLSGrid.hpp"

#include "envire/Core.hpp"
#include <vizkit/Uncertainty.hpp>

using namespace envire;

BOOST_AUTO_TEST_CASE( mlsviz_test ) 
{
    QtThreadedWidget<vizkit::QVizkitWidget> app;
    vizkit::EnvireVisualization envViz;
    app.start();
    app.getWidget()->addDataHandler( &envViz );

    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );
    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls );

    FrameNode *fm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( -5, -5, 0 ) ) );
    env->getRootNode()->addChild( fm );
    mls->setFrameNode( fm );

    envViz.updateData( env.get() );

    for(int i=0;i<5000 && app.isRunning();i++)
    {
	double r = i/10.0;
	for(int x=0;x<100;x++)
	{
	    for(int y=0;y<100;y++)
	    {
		MultiLevelSurfaceGrid::SurfacePatch p( cos( (x+r)/10.0 ) * sin( (y+r)/10.0 ), 0.1, 0, true );

		MultiLevelSurfaceGrid::iterator it = mls->beginCell(x,y);
		if( it == mls->endCell() )
		{
		    mls->insertHead( x, y, p );
		}
		else 
		{
		    *it = p;
		}
	    }
	}

	fm->setTransform( Eigen::Affine3d( Eigen::AngleAxisd( r/50.0, Eigen::Vector3d::UnitZ() ) * Eigen::Translation3d( -5, -5, 0 )  ) );
	mls->itemModified();

	usleep(1000);
    }
}

