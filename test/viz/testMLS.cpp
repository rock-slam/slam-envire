#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"

#include "envire/Core.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/maps/Pointcloud.hpp"
#include "envire/operators/MLSProjection.hpp"

using namespace envire;

BOOST_AUTO_TEST_CASE( mlsmatch_test ) 
{
    QtThreadedWidget<vizkit::QVizkitWidget> app;
    vizkit::EnvireVisualization envViz;
    app.start();
    app.getWidget()->addDataHandler( &envViz );

    boost::scoped_ptr<Environment> env( new Environment() );
    envViz.updateData( env.get() );

    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls );

    FrameNode *fm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( -5, -5, 0 ) ) );
    env->getRootNode()->addChild( fm );
    mls->setFrameNode( fm );
    for( int i=0; i<100; i++ )
    {
	for( int j=0; j<100; j++ )
	{
	    double h = 0.0;
	    mls->insertTail( i, j, MultiLevelSurfaceGrid::SurfacePatch( h, 0.05 ) );
	}
    }
    mls->itemModified();

    MultiLevelSurfaceGrid *mls2 = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls2 );

    FrameNode *fm2 = fm->clone();
    env->getRootNode()->addChild( fm2 );
    mls2->setFrameNode( fm2 );

    for(int i=0;i<500 && app.isRunning();i++)
    {
	mls2->clear();
	for( int m=48; m<52; m++ )
	{
	    for( int n=40; n<60; n++ )
	    {
		double r = i * 0.02;
		double h = std::max( sin(m*0.1+n*0.2), r  );
		mls2->insertTail( m, n, MultiLevelSurfaceGrid::SurfacePatch( h, 0.05 ) );
	    }
	}
	mls2->itemModified();
	std::pair<double, double> res = mls->matchHeight( *mls2 );
	std::cout << "diff : " << res.first << " var: " << res.second << std::endl;
	fm2->setTransform( Eigen::Affine3d( Eigen::Translation3d( 0, 0, -res.first ) ) );

	usleep( 1000*1000 );
    }
}
