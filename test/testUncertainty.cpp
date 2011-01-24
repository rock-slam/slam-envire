#define BOOST_TEST_MODULE UncertaintyTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"
#include "maps/MultiLevelSurfaceGrid.hpp"
#include "maps/Pointcloud.hpp"
#include "operators/MLSProjection.hpp"

#include "Core.hpp"
#include "Uncertainty.hpp"

using namespace envire;

namespace vizkit
{
class UncertaintyVisualization : public VizPlugin<envire::PointWithUncertainty>
{
public:
    osg::ref_ptr<osg::Node> createMainNode()
    {
	m_uncertainty = new Uncertainty();
	m_uncertainty->setNumSamples( 10000 );
	return m_uncertainty;
    }

    void updateMainNode(osg::Node* node)
    {
	m_uncertainty->setMean( point.getPoint() );
	m_uncertainty->setCovariance( point.getCovariance() );
    }

    void updateDataIntern(const envire::PointWithUncertainty &point)
    {
	this->point = point;
    }

private:
    envire::PointWithUncertainty point;
    osg::ref_ptr<Uncertainty> m_uncertainty;
};
}

BOOST_AUTO_TEST_CASE( uncertainty_test ) 
{
    QtThreadedWidget<vizkit::QVizkitWidget> app;
    vizkit::UncertaintyVisualization viz;
    app.start();
    app.widget->addDataHandler( &viz );

    for(int i=0;i<5000 && app.isRunning();i++)
    {
	double r = i/5000.0;
	PointWithUncertainty p(
		Eigen::Vector3d( 0.0, 0, 0 ),
		Eigen::Vector3d( 0.01, 0.01, 0.01 ).cwise().square().asDiagonal() );
	
	Eigen::Matrix<double,6,6> lt1; 
	lt1 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t1(
		Eigen::Transform3d(Eigen::Translation3d(Eigen::Vector3d(r,0,0))),
		lt1.cwise().square() );

	Eigen::Matrix<double,6,6> lt2; 
	lt2 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t2(
		Eigen::Transform3d(Eigen::Transform3d::Identity()),
		lt2.cwise().square() );

	PointWithUncertainty pp = t2 * t1 * p;

	viz.updateData( pp );

	usleep(1000);
    }
}

BOOST_AUTO_TEST_CASE( uncertaintymls_test ) 
{
    QtThreadedWidget<vizkit::QVizkitWidget> app;
    vizkit::EnvireVisualization envViz;
    vizkit::UncertaintyVisualization viz;
    app.start();
    app.widget->addDataHandler( &envViz );
    app.widget->addDataHandler( &viz );

    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );
    envViz.updateData( env.get() );

    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls );

    FrameNode *fm = new FrameNode( Eigen::Transform3d( Eigen::Translation3d( -5, -5, 0 ) ) );
    env->getRootNode()->addChild( fm );
    mls->setFrameNode( fm );

    envire::Pointcloud* pc = new envire::Pointcloud();
    env->attachItem( pc );
    std::vector<double> &vars = pc->getVertexData<double>( Pointcloud::VERTEX_VARIANCE );
    for( int i=0; i<100; i++ )
    {
	const double r = i/100.0;
	pc->vertices.push_back( Eigen::Vector3d( r-0.5, 1.0, 0 ) );
	vars.push_back( 0.001 );
    }

    FrameNode *pcfm = new FrameNode( Eigen::Transform3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->getRootNode()->addChild( pcfm );
    pc->setFrameNode( pcfm );

    envire::MLSProjection *proj = new envire::MLSProjection();
    env->attachItem( proj );
    proj->addInput( pc );
    proj->addOutput( mls );
    proj->useUncertainty( true );

    for(int i=0;i<500 && app.isRunning();i++)
    {
	double r = i*0.1;
	Eigen::Matrix<double,6,1> c;
	//c << 0, M_PI/8.0, 0, 0, 0, 0;
	c << 0, 0, 0, 0, 0, 1;
	pcfm->setTransform( TransformWithUncertainty( 
		    Eigen::Transform3d( Eigen::Translation3d( r, r, 0 ) ),
		   c.cwise().square().asDiagonal() ) );
	proj->updateAll();

	PointWithUncertainty p( pc->vertices[0], Eigen::Matrix3d::Identity() * vars[0] );
	viz.updateData( pcfm->getTransformWithUncertainty() * p );

	usleep(1000*1000);
    }
}

