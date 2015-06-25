#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"

#include "envire/Core.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/maps/Pointcloud.hpp"
#include "envire/operators/MLSProjection.hpp"

using namespace envire;

BOOST_AUTO_TEST_CASE( mlsmatch_test ) 
{
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    envire::EnvireVisualization envViz;
    app.start();
    app.getWidget()->addPlugin( &envViz );

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


BOOST_AUTO_TEST_CASE( mlsnegative_test ) 
{
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    envire::EnvireVisualization envViz;
    app.start();
    app.getWidget()->addPlugin( &envViz );

    boost::scoped_ptr<Environment> env( new Environment() );
    envViz.updateData( env.get() );

    // setup a transformation chain
    // pointcloud -> mls

    Pointcloud *pc = new Pointcloud();
    env->attachItem( pc );
    FrameNode *pcfn = new FrameNode( 
	    Eigen::Affine3d( Eigen::Translation3d( 0, 0.5, 2 ) ) );
    env->getRootNode()->addChild( pcfn );
    pc->setFrameNode( pcfn );

    Pointcloud *pc2 = new Pointcloud();
    env->attachItem( pc2 );
    FrameNode *pc2fn = new FrameNode( 
	    Eigen::Affine3d( Eigen::Translation3d( 0, -0.5, 1 ) ) );
    env->getRootNode()->addChild( pc2fn );
    pc2->setFrameNode( pc2fn );

    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.5, 0.5, -25, -25);
    env->attachItem( mls );
    FrameNode *mlsfn = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->getRootNode()->addChild( mlsfn );
    mls->setFrameNode( mlsfn );

    MLSProjection *mlsp = new MLSProjection();
    mlsp->useNegativeInformation( true );
    env->attachItem( mlsp );
    mlsp->addInput( pc );
    mlsp->addInput( pc2 );
    mlsp->addOutput( mls );

    // fill the pointcloud 
    for(int i=0; i<20; i++)
    {
	for(int j=0; j<20; j++)
	{
	    if( i == 0 || j == 0 || j == 19 )
	    {
		pc->vertices.push_back( Eigen::Vector3d( j / 4.0 - 2.5, 2.0, i / 4.0 - 2.5 ) );
	    }
	}
    }

    // fill the 2. pointcloud 
    for(int i=0; i<20; i++)
    {
	for(int j=0; j<20; j++)
	{
	    if( i == 0 || j == 0 || j == 19 )
	    {
		pc2->vertices.push_back( Eigen::Vector3d( 2.0*(j / 4.0 - 2.5), 2.0, i / 4.0 - 2.5 ) );
	    }
	}
    }

    mlsp->updateAll();

    for(int i=0;i<500 && app.isRunning();i++)
    {
	usleep( 1000*1000 );

	Eigen::Affine3d t = pcfn->getTransform();
	t.translation() += Eigen::Vector3d( 0, 0.5, 0 ); 
	pcfn->setTransform( t );

	Eigen::Affine3d t2 = pc2fn->getTransform();
	t2.translation() += Eigen::Vector3d( 0, 0.5, 0 ); 
	pc2fn->setTransform( t2 );

	mlsp->updateAll();

        GridBase::Position origin;
        if( mls->toGrid( Eigen::Vector2d(-1.5, 3.0), origin ) )
        {       
                std::cerr << "origin: (" << origin.x << "," << origin.y << ")" << std::endl;
                for(MultiLevelSurfaceGrid::iterator cit = mls->beginCell(origin.x, origin.y); cit != mls->endCell(); cit++ )
	        {
                    std::cerr << cit->getTypeName() << ": idx: " <<  cit->update_idx << ", mean: " << cit->mean << ", height: " << cit->height << std::endl;
                }
        }
    }
}

BOOST_AUTO_TEST_CASE( mlsslope_test ) 
{
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    envire::EnvireVisualization envViz;
    app.start();
    app.getWidget()->addPlugin( &envViz );

    boost::scoped_ptr<Environment> env( new Environment() );
    envViz.updateData( env.get() );

    // setup a transformation chain
    // pointcloud -> mls

    Pointcloud *pc = new Pointcloud();
    env->attachItem( pc );
    FrameNode *pcfn = new FrameNode( 
        Eigen::Affine3d( Eigen::Translation3d( 0, 0.5, 2 ) ) );
    env->getRootNode()->addChild( pcfn );
    pc->setFrameNode( pcfn );

    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.5, 0.5, -25, -25);
    envire::MLSConfiguration& config = mls->getConfig();
    config.updateModel = envire::MLSConfiguration::SLOPE;
    env->attachItem( mls );
    FrameNode *mlsfn = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->getRootNode()->addChild( mlsfn );
    mls->setFrameNode( mlsfn );

    MLSProjection *mlsp = new MLSProjection();
    env->attachItem( mlsp );
    mlsp->addInput( pc );
    mlsp->addOutput( mls );

    // fill the pointcloud 
    for(int i=0; i<20; i++)
    {
        for(int j=0; j<20; j++)
        {
            if( i == 0 || j == 0 || j == 19 )
            {
                pc->vertices.push_back( Eigen::Vector3d( j / 4.0 - 2.5, 2.0, i / 4.0 - 2.5 ) );
            }
        }
    }

    mlsp->updateAll();

    for(int i=0;i<500 && app.isRunning();i++)
    {
	usleep( 1000*1000 );

	Eigen::Affine3d last_transform = pcfn->getTransform();
        Eigen::Affine3d next_transform(Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
        next_transform.translation() = Eigen::Vector3d( 0, 0.1, 0 );
	pcfn->setTransform( last_transform * next_transform );

        env->itemModified(pc);
	mlsp->updateAll();
    }
}
