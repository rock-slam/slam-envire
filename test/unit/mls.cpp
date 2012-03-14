#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include "envire/Core.hpp"

#include "envire/maps/MLSGrid.hpp"
#include "envire/operators/MLSProjection.hpp"
#include "envire/operators/MergeMLS.hpp"

using namespace envire;

BOOST_AUTO_TEST_CASE( multilevelsurfacegrid ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(10, 10, 0.1, 0.1);
    env->attachItem( mls );

    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 1.0, 0.1, 0, MLSGrid::SurfacePatch::HORIZONTAL ) );
    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 2.0, 0.1, 0.5, MLSGrid::SurfacePatch::VERTICAL ) );

    mls->insertHead( 2,1, MultiLevelSurfaceGrid::SurfacePatch( 3.0, 0.1, 0.5, MLSGrid::SurfacePatch::VERTICAL ) );

    MultiLevelSurfaceGrid::iterator it = mls->beginCell(0,0);
    BOOST_CHECK_EQUAL( it->mean, 2.0 );
    it++;
    BOOST_CHECK_EQUAL( (*it).mean, 1.0 );
    ++it;
    BOOST_CHECK_EQUAL( it, mls->endCell() );

    MultiLevelSurfaceGrid::iterator it2 = mls->beginCell(2,1);
    BOOST_CHECK_EQUAL( it2->mean, 3.0 );
    it2++;
    BOOST_CHECK_EQUAL( it2, mls->endCell() );
}

BOOST_AUTO_TEST_CASE( mlsprojection_test ) 
{
    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );

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

    for(int i=0;i<500;i++)
    {
	double r = i*0.1;
	Eigen::Matrix<double,6,1> c;
	//c << 0, M_PI/8.0, 0, 0, 0, 0;
	c << 0, r*0.1, 0, 0, 0, 0;
	pcfm->setTransform( TransformWithUncertainty( 
		    Eigen::Affine3d( Eigen::Translation3d( 0, r+0.05, 0 ) ),
		   c.array().square().matrix().asDiagonal() ) );
	proj->updateAll();
    }
}

BOOST_AUTO_TEST_CASE( mlsmerge_test ) 
{
    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );

    MultiLevelSurfaceGrid *mls1 = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls1 );

    MultiLevelSurfaceGrid *mls2 = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    env->attachItem( mls2 );

    for( size_t m=0; m<100; m++ )
    {
	for( size_t n=0; n<100; n++ )
	{
	    double h1 = cos(n*0.1)*sin(n*0.1);
	    double h2 = cos(n*0.2)*sin(n*0.3);
	    mls1->insertHead( m, n, MultiLevelSurfaceGrid::SurfacePatch( h1, 0.1 ) );
	    mls2->insertHead( m, n, MultiLevelSurfaceGrid::SurfacePatch( h2, 0.1 ) );
	}
    }

    FrameNode *fm = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( -5, -5, 0 ) ) );
    env->getRootNode()->addChild( fm );
    mls1->setFrameNode( fm );

    FrameNode *fm2 = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( -5, -5, 1 ) ) );
    env->getRootNode()->addChild( fm2 );
    mls2->setFrameNode( fm2 );

    envire::MergeMLS *merge = new envire::MergeMLS();
    env->attachItem( merge );
    merge->addInput( mls1 );
    merge->addOutput( mls2 );
    merge->updateAll();

}
