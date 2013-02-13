#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include "envire/Core.hpp"

#include "envire/maps/MLSGrid.hpp"
#include "envire/operators/MLSProjection.hpp"
#include "envire/operators/MergeMLS.hpp"

#include "envire/tools/ListGrid.hpp"

#include <base/timemark.h>

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
    BOOST_CHECK( it == mls->endCell() );

    MultiLevelSurfaceGrid::iterator it2 = mls->beginCell(2,1);
    BOOST_CHECK_EQUAL( it2->mean, 3.0 );
    it2++;
    BOOST_CHECK( it2 == mls->endCell() );
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

BOOST_AUTO_TEST_CASE( gridaligned_test ) 
{
    // set up test environment
    boost::scoped_ptr<Environment> env( new Environment() );

    MultiLevelSurfaceGrid *mls1 = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    FrameNode *fn1 = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->addChild( env->getRootNode(), fn1 ); 
    env->setFrameNode( mls1, fn1 );

    MultiLevelSurfaceGrid *mls2 = new MultiLevelSurfaceGrid(100, 100, 0.1, 0.1);
    FrameNode *fn2 = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0, 0, 0 ) ) );
    env->addChild( env->getRootNode(), fn2 ); 
    env->setFrameNode( mls2, fn2 );

    MultiLevelSurfaceGrid *mls3 = new MultiLevelSurfaceGrid(200, 100, 0.1, 0.1);
    FrameNode *fn3 = new FrameNode( Eigen::Affine3d( Eigen::Translation3d( 0.1, 0, 0 ) ) );
    env->addChild( env->getRootNode(), fn3 ); 
    env->setFrameNode( mls3, fn3 );

    BOOST_CHECK( mls1->isAlignedWith( *mls2 )  );
    BOOST_CHECK( !mls2->isAlignedWith( *mls3 )  );
    BOOST_CHECK( mls2->isCellAlignedWith( *mls3 )  );

    fn3->setTransform( Eigen::Affine3d( Eigen::Translation3d( 0.15, 0, 0 ) ) );
    BOOST_CHECK( !mls2->isCellAlignedWith( *mls3 ) );
}

inline void populateRandom( envire::MLSGrid::Ptr grid, const size_t count )
{
    const size_t gridsize_x = grid->getCellSizeX();
    const size_t gridsize_y = grid->getCellSizeY();

    for( size_t n=0; n<count; n++ )
    {
	size_t x = rand() % gridsize_x;
	size_t y = rand() % gridsize_y;
	MLSGrid::SurfacePatch p( rand()%100 / 100.0, rand()%100 / 100.0 );
	grid->updateCell( x, y, p );
    }
}

BOOST_AUTO_TEST_CASE( profiling_test ) 
{
    // randomly populate the grid
    srand(0);

    size_t grid_size = 1000;
    MLSGrid::Ptr mls( new MLSGrid(grid_size, grid_size, 0.1, 0.1) );

    base::TimeMark tmls("Populate MLS 10 x 1000000");
    for( int i=0; i<10; i++ )
    {
	mls->clear();
	populateRandom( mls, 1000000 );
    }
    std::cout 
	<< "cellcount: " << mls->getCellCount() 
	<< " " << tmls 
	<< std::endl;

}

struct Integer
{
    int v;
    Integer(int i) : v(i) {};
    operator int() const { return v; }
};

BOOST_AUTO_TEST_CASE( list_grid )
{
    ListGrid<Integer> lg( 10, 10 );

    lg.insertHead( 1, 1, 10 );
    lg.insertTail( 1, 1, 20 );

    {
	ListGrid<Integer>::iterator it = lg.beginCell( 1, 1 );
	BOOST_CHECK_EQUAL( *(it++), 10 );
	BOOST_CHECK_EQUAL( *(it++), 20 );
	BOOST_CHECK( it == lg.endCell() );
    }

    {
	const ListGrid<Integer>& clg( lg );
	// and the same thing for const
	ListGrid<Integer>::const_iterator it = clg.beginCell( 1, 1 );
	BOOST_CHECK_EQUAL( *(it++), 10 );
	BOOST_CHECK_EQUAL( *(it++), 20 );
	BOOST_CHECK( it == clg.endCell() );
    }
}


