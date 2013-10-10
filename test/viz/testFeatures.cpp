#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "EnvireWidget.hpp"
#include "envire/maps/Featurecloud.hpp"

using namespace envire;

BOOST_AUTO_TEST_CASE( mlsviz_test ) 
{
    // set up test environment
    QtThreadedWidget<envire::EnvireWidget> app;
    app.start();
    Environment* env = app.getWidget()->getEnvironment();

    Featurecloud *fc = new Featurecloud();
    for( int i=0; i<20; i++ )
    {
	fc->vertices.push_back( Eigen::Vector3d::Random() * 10.0 );
	std::cout <<  (Eigen::Vector3d::Random() * 10.0).transpose() << std::endl;
	envire::KeyPoint kp;
	kp.size = (rand()%100) * 0.05;
	std::cout << kp.size << std::endl;
	fc->keypoints.push_back( kp );
    }
    env->attachItem( fc );

    FrameNode *fm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->getRootNode()->addChild( fm );
    fc->setFrameNode( fm );

    usleep(1000*1000*10);
}

