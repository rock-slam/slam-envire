#define BOOST_TEST_MODULE MLSToPointCloudTest

#include <boost/shared_ptr.hpp>
#include <boost/test/included/unit_test.hpp>
#include <envire/operators/MLSToPointCloud.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Pointcloud.hpp>

using namespace envire;

BOOST_AUTO_TEST_CASE( test_empty )
{
    boost::shared_ptr<Environment> env(new Environment);
    MLSGrid* mls_grid = new MLSGrid(100, 100, 0.1, 0.1);
    Pointcloud* pc = new Pointcloud();
    MLSToPointCloud* mlsToPCptr = new MLSToPointCloud();
    
    env->attachItem( mls_grid );
    env->attachItem( pc );
    
    FrameNode *fm = new FrameNode();
    env->getRootNode()->addChild( fm );
    mls_grid->setFrameNode( fm );
    pc->setFrameNode( fm );
    
    env->addInput(mlsToPCptr, mls_grid);
    env->addOutput(mlsToPCptr, pc);
    
    env->updateOperators();
    
    BOOST_CHECK( pc->vertices.size() == 0 );
    BOOST_CHECK( mlsToPCptr->getEnvironment() == env.get() );
    
    MLSGrid* out = static_cast<MLSGrid*>(mlsToPCptr->getInput<MLSGrid*>());
    BOOST_CHECK( out == mls_grid );
    

    
}



BOOST_AUTO_TEST_CASE( test_one )
{
	srand(std::time(NULL)); //initializing rand seed
	boost::shared_ptr<Environment> env(new Environment);
	MLSGrid* mls_grid = new MLSGrid(100, 100, 1, 1);
	Pointcloud* pc = new Pointcloud();
	MLSToPointCloud* mlsToPCptr = new MLSToPointCloud();

	env->attachItem( mls_grid );
	env->attachItem( pc );

	FrameNode* fm = new FrameNode();
	env->getRootNode()->addChild( fm );
	mls_grid->setFrameNode( fm );
	pc->setFrameNode( fm );

	env->addInput(mlsToPCptr, mls_grid);
	env->addOutput(mlsToPCptr, pc);

	int x = rand() % 100;
	int y = rand() % 100;
	int h = rand() % 10;
	mls_grid->insertTail((size_t)x, (size_t)y, MLSGrid::SurfacePatch( h, 0.5 ));

	env->updateOperators();
	BOOST_CHECK( pc->vertices.size() == mls_grid->getCellCount() );
	BOOST_CHECK( pc->vertices.size() == 1);

	std::vector< Eigen::Vector3d >::iterator it;
	it = pc->vertices.begin();

	BOOST_CHECK((*it).x() == x + 0.5 );
	BOOST_CHECK((*it).y() == y + 0.5 );
	BOOST_CHECK((*it).z() == h );
}



BOOST_AUTO_TEST_CASE( test_full )
{
	srand(std::time(NULL)); //initializing rand seed
	boost::shared_ptr<Environment> env(new Environment);
	MLSGrid* mls_grid = new MLSGrid(242, 274, 0.1, 0.1);
	Pointcloud* pc = new Pointcloud();
	MLSToPointCloud* mlsToPCptr = new MLSToPointCloud();
	
	env->attachItem( mls_grid );
	env->attachItem( pc );
	
	FrameNode* fm = new FrameNode();
	env->getRootNode()->addChild( fm );
	mls_grid->setFrameNode( fm );
	pc->setFrameNode( fm );
	
	env->addInput(mlsToPCptr, mls_grid);
	env->addOutput(mlsToPCptr, pc);
	
	
	for(size_t x = 0; x < mls_grid->getCellSizeX(); x++)
	{
		for(size_t y = 0; y < mls_grid->getCellSizeY(); y++)
		{
			int h = rand() % 10;
			mls_grid->insertTail(x, y, MLSGrid::SurfacePatch( h, 0.05 ));
		}
			
	}
	
	env->updateOperators();
	
	BOOST_CHECK( pc->vertices.size() == mls_grid->getCellCount() );

	std::vector< Eigen::Vector3d >::iterator it;
	for(it = pc->vertices.begin(); it != pc->vertices.end(); it++)
	{
		double a = 0, b = 0;
		MLSGrid::SurfacePatch* patch = mls_grid->get(*it, a, b);
		BOOST_CHECK( patch->getMean() == (*it)[2] );
	}
}

BOOST_AUTO_TEST_CASE( test_vertical_patches )
{
	boost::shared_ptr<Environment> env(new Environment);
	MLSGrid* mls_grid = new MLSGrid(100, 100, 0.1, 0.1);
	Pointcloud* pc = new Pointcloud();
	MLSToPointCloud* mlsToPCptr = new MLSToPointCloud();
	
	env->attachItem( mls_grid );
	env->attachItem( pc );
	
	FrameNode* fm = new FrameNode();
	env->getRootNode()->addChild( fm );
	mls_grid->setFrameNode( fm );
	pc->setFrameNode( fm );
	
	env->addInput(mlsToPCptr, mls_grid);
	env->addOutput(mlsToPCptr, pc);
	
	
	for(size_t x = 0; x < mls_grid->getCellSizeX(); x++)
	{
		for(size_t y = 0; y < mls_grid->getCellSizeY(); y++)
		{
			int h = rand() % 10;
			mls_grid->insertTail(x, y, MLSGrid::SurfacePatch( 0, 0.5 , h, SurfacePatch::VERTICAL ) );
		}
			
	}
	
	env->updateOperators();
	
	std::vector< Eigen::Vector3d >::iterator it;
	for(it = pc->vertices.begin(); it != pc->vertices.end(); it++)
	{
		double a,b;
		MLSGrid::SurfacePatch* patch = mls_grid->get(*it, a, b);
		BOOST_CHECK( patch->getMean() == 0 );
		BOOST_CHECK( patch->isVertical() && !patch->isHorizontal() && !patch->isNegative() );
		BOOST_CHECK( patch->getHeight() < 10 && patch->getHeight() >= 0 );

	}
	
	
}


BOOST_AUTO_TEST_CASE( test_negative_gap_double )
{
	srand(std::time(NULL)); //initializing rand seed
	boost::shared_ptr<Environment> env(new Environment);
	MLSGrid* mls_grid = new MLSGrid(100, 100, 0.1, 0.1);
	Pointcloud* pc = new Pointcloud();
	MLSToPointCloud* mlsToPCptr = new MLSToPointCloud();
	
	env->attachItem( mls_grid );
	env->attachItem( pc );
	
	FrameNode* fm = new FrameNode();
	env->getRootNode()->addChild( fm );
	mls_grid->setFrameNode( fm );
	pc->setFrameNode( fm );
	
	env->addInput(mlsToPCptr, mls_grid);
	env->addOutput(mlsToPCptr, pc);
	
	
	for(size_t x = 0; x < mls_grid->getCellSizeX(); x++)
	{
		for(size_t y = 0; y < mls_grid->getCellSizeY(); y++)
		{
			if (rand()% 2)
			{
			      double h = (rand() % 200 - 100) / 10.0;
			      mls_grid->insertTail(x, y, MLSGrid::SurfacePatch( h, 0.05 ));
			}
		}
	}
	
	env->updateOperators();
	
	BOOST_CHECK( pc->vertices.size() == mls_grid->getCellCount() );

	std::vector< Eigen::Vector3d >::iterator it;
	for(it = pc->vertices.begin(); it != pc->vertices.end(); it++)
	{
		double a = 0, b = 0;
		mls_grid->get(*it, a, b);
		BOOST_CHECK( a == it->z() );
	}
}

