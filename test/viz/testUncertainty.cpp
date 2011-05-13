#define BOOST_TEST_MODULE UncertaintyTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/maps/Pointcloud.hpp"
#include "envire/operators/MLSProjection.hpp"

#include "envire/Core.hpp"
#include <vizkit/Uncertainty.hpp>

using namespace envire;

namespace vizkit
{
class UncertaintyVisualization : public VizPlugin<envire::PointWithUncertainty>
{
public:
    UncertaintyVisualization()
	: point( Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero() ) {}
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
    app.getWidget()->addDataHandler( &viz );

    for(int i=0;i<5000 && app.isRunning();i++)
    {
	double r = i/5000.0;
	PointWithUncertainty p(
		Eigen::Vector3d( 0.0, 0, 0 ),
		Eigen::Vector3d( 0.01, 0.01, 0.01 ).array().square().matrix().asDiagonal() );
	
	Eigen::Matrix<double,6,6> lt1; 
	lt1 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t1(
		Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(r,0,0))),
		lt1.array().square().matrix() );

	Eigen::Matrix<double,6,6> lt2; 
	lt2 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t2(
		Eigen::Affine3d(Eigen::Affine3d::Identity()),
		lt2.array().square().matrix() );

	PointWithUncertainty pp = t2 * t1 * p;

	viz.updateData( pp );

	usleep(1000);
    }
}

std::ostream& operator<<( std::ostream &os, const envire::TransformWithUncertainty& t )
{
    os << "transform:" << std::endl;
    os << t.getTransform().matrix() << std::endl;
    os << "uncertainty:" << std::endl;
    os << t.getCovariance() << std::endl;
    os << std::endl;
    return os;
}

BOOST_AUTO_TEST_CASE( mlsmerge_test ) 
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

    for( int i=0; i<10; i++ )
    {
	for( int j=0; j<10; j++ )
	{
	    double h = (i*j)/20.0;
	    mls->insertTail( 50, 45+i, MultiLevelSurfaceGrid::SurfacePatch( h, 0.05 ) );
	}
    }
    mls->itemModified();

    for(int i=0;i<500 && app.isRunning();i++)
    {
	int n = i%10;
	double h = i/10.0;

	mls->updateCell( 50, 45+n, MultiLevelSurfaceGrid::SurfacePatch( h, 0.05 ) );
	mls->itemModified();

	usleep( 1000*1000 );
	std::cout << mls->getCellCount() << std::endl;
    }
}

BOOST_AUTO_TEST_CASE( uncertaintymls_test ) 
{
    const size_t uncertainty_points = 10;
    QtThreadedWidget<vizkit::QVizkitWidget> app;
    vizkit::EnvireVisualization envViz;
    vizkit::UncertaintyVisualization viz[uncertainty_points];
    app.start();
    app.getWidget()->addDataHandler( &envViz );
    for(size_t i=0;i<uncertainty_points;i++)
	app.getWidget()->addDataHandler( &viz[i] );

    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );
    envViz.updateData( env.get() );

    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls );

    FrameNode *fm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( -5, -5, 0 ) ) );
    env->getRootNode()->addChild( fm );
    mls->setFrameNode( fm );

    envire::Pointcloud* pc = new envire::Pointcloud();
    env->attachItem( pc );
    std::vector<double> &vars = pc->getVertexData<double>( Pointcloud::VERTEX_VARIANCE );
    for( int i=0; i<100; i++ )
    {
	const double r = i/100.0;
	pc->vertices.push_back( Eigen::Vector3d( r-0.5, 1.0, 0 ) );
	vars.push_back( 0 );
    }

    FrameNode *pcfm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
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
	c << 0, 0, 0, 0, 0, 0.2;
	pcfm->setTransform( TransformWithUncertainty( 
		    //Eigen::Affine3d( Eigen::Translation3d( 0, r+0.05, 0 ) ),
		    Eigen::Affine3d( Eigen::Translation3d( 0, 0, r+0.05 ) ),
		   c.array().square().matrix().asDiagonal() ) );
	proj->updateAll();

	TransformWithUncertainty fm2g = env->relativeTransformWithUncertainty( pcfm, env->getRootNode() );
	//TransformWithUncertainty fm2g = env->getRootNode()->getTransformWithUncertainty().inverse() * pcfm->getTransformWithUncertainty();

	for(size_t i=0;i<uncertainty_points;i++)
	{
	    PointWithUncertainty p( pc->vertices[i*10], Eigen::Matrix3d::Identity() * vars[i*10] );
	    viz[i].updateData( fm2g * p );
	}

	std::cout << mls->getCellCount() << std::endl;

	usleep(1000*1000);
    }
}

