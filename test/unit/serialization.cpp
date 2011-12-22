#define BOOST_TEST_MODULE SerializationTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include "envire/Core.hpp"

#include "envire/maps/MLSGrid.hpp"

using namespace envire;

struct SerializationPath
{
    std::string serialization_test_path;

    SerializationPath()
    {
        std::string current_path(__FILE__);
        current_path = current_path.substr(0,current_path.find_last_of("/"));
        current_path = current_path.substr(0,current_path.find_last_of("/"));
        current_path = current_path.substr(0,current_path.find_last_of("/"));
        serialization_test_path = current_path.append("/build/test");
    }

    ~SerializationPath()
    {
        
    }
};

BOOST_FIXTURE_TEST_SUITE(SerializationTest, SerializationPath)

BOOST_AUTO_TEST_CASE( environment_serialization )
{
    boost::scoped_ptr<Environment> env( new Environment() );

    // create some child framenodes
    FrameNode *fn1, *fn2, *fn3;
    fn1 = new FrameNode();
    fn1->setTransform( 
        Eigen::Affine3d(Eigen::Translation3d( 0.0, 0.0, 0.5 )) );
    fn2 = new FrameNode();
    fn2->setTransform( 
        Eigen::Affine3d(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0 )));
    fn3 = new FrameNode();
    
    // attach explicitely
    env->attachItem( fn1 );
    env->addChild( env->getRootNode(), fn1 );
    env->addChild( fn1, fn2 );
    env->addChild( env->getRootNode(), fn3 );

    std::string path(serialization_test_path);
    env->serialize(path);

    // now try to parse the thing again
    boost::scoped_ptr<Environment> env2(Environment::unserialize(path));
    
    std::list<FrameNode*> root_childs = env2->getChildren(env2->getRootNode());
    std::list<FrameNode*> fn1_childs;
    std::list<FrameNode*> fn3_childs;
    
    // check if the root note has two childs
    BOOST_CHECK_EQUAL(root_childs.size(), 2);
    // check if the childs are equal to fn1 and fn3
    if(fn1->getUniqueId() == root_childs.front()->getUniqueId())
    {
        // fn1' is the first element of the list
        fn1_childs = env2->getChildren(root_childs.front());
        fn3_childs = env2->getChildren(root_childs.back());
        BOOST_CHECK_EQUAL(fn1->getTransform().matrix().rows() * fn1->getTransform().matrix().cols(), root_childs.front()->getTransform().matrix().cwiseEqual(fn1->getTransform().matrix()).count());
        BOOST_CHECK_EQUAL(fn3->getTransform().matrix().rows() * fn3->getTransform().matrix().cols(), root_childs.back()->getTransform().matrix().cwiseEqual(fn3->getTransform().matrix()).count());
    }
    else
    {
        // fn1' is the second element of the list
        fn1_childs = env2->getChildren(root_childs.back());
        fn3_childs = env2->getChildren(root_childs.front());
        BOOST_CHECK_EQUAL(fn3->getTransform().matrix().rows() * fn3->getTransform().matrix().cols(), root_childs.front()->getTransform().matrix().cwiseEqual(fn3->getTransform().matrix()).count());
        BOOST_CHECK_EQUAL(fn1->getTransform().matrix().rows() * fn1->getTransform().matrix().cols(), root_childs.back()->getTransform().matrix().cwiseEqual(fn1->getTransform().matrix()).count());
    }
    
    // check if fn1' has one child and fn3' has no childs
    BOOST_CHECK_EQUAL(fn1_childs.size(), 1);
    BOOST_CHECK_EQUAL(fn3_childs.size(), 0);
    
    // check if the child of fn1' is equal to fn2
    BOOST_CHECK_EQUAL(fn2->getTransform().matrix().rows() * fn2->getTransform().matrix().cols(), fn1_childs.front()->getTransform().matrix().cwiseEqual(fn2->getTransform().matrix()).count());
    
    // check if fn2' has no childs
    BOOST_CHECK_EQUAL(env2->getChildren(fn1_childs.front()).size(),0);
}

BOOST_AUTO_TEST_CASE( multilevelsurfacegrid_serialization ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(10, 10, 0.1, 0.1);
    env->attachItem( mls );

    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 1.0, 0.1, 0, true ) );
    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 2.0, 0.1, 0.5, false ) );

    mls->insertHead( 2,1, MultiLevelSurfaceGrid::SurfacePatch( 3.0, 0.1, 0.5, false ) );

    env->serialize(serialization_test_path);

    boost::scoped_ptr<Environment> env2(Environment::unserialize(serialization_test_path));
    MultiLevelSurfaceGrid *mls2 = env2->getItems<MultiLevelSurfaceGrid>().front();

    MultiLevelSurfaceGrid::iterator it = mls2->beginCell(0,0);
    BOOST_CHECK_EQUAL( it->mean, 2.0 );
    it++;
    BOOST_CHECK_EQUAL( (*it).mean, 1.0 );
    it++;
    BOOST_CHECK_EQUAL( it, mls2->endCell() );
    
    MultiLevelSurfaceGrid::iterator it2 = mls2->beginCell(2,1);
    BOOST_CHECK_EQUAL( it2->mean, 3.0 );
    it2++;
    BOOST_CHECK_EQUAL( it2, mls2->endCell() );
}

BOOST_AUTO_TEST_CASE( framenode_binitem_serialization ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    FrameNode *fn = new FrameNode();
    fn->setTransform( Eigen::Affine3d(Eigen::Translation3d( 0.0, 0.0, 0.5 )) );
    env->attachItem( fn );
    
    BinarySerialization serialization;
    EnvireBinaryItem bin_item;
    
    serialization.serializeBinaryItem(fn, bin_item);
    
    EnvironmentItem *new_item = serialization.unserializeBinaryItem(bin_item);
    BOOST_CHECK_EQUAL(new_item->getClassName(), fn->getClassName());
    BOOST_CHECK_EQUAL(new_item->getUniqueId(), fn->getUniqueId());
    
    FrameNode *fn_new = dynamic_cast<FrameNode*>(new_item);
    
    BOOST_CHECK_EQUAL(fn->getTransform().matrix().rows() * fn->getTransform().matrix().cols(), fn_new->getTransform().matrix().cwiseEqual(fn->getTransform().matrix()).count());
}

BOOST_AUTO_TEST_CASE( multilevelsurfacegrid_binitem_serialization ) 
{
    boost::scoped_ptr<Environment> env( new Environment() );
    MultiLevelSurfaceGrid *mls = new MultiLevelSurfaceGrid(10, 10, 0.1, 0.1);
    env->attachItem( mls );

    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 1.0, 0.1, 0, true ) );
    mls->insertHead( 0,0, MultiLevelSurfaceGrid::SurfacePatch( 2.0, 0.1, 0.5, false ) );
    mls->insertHead( 2,1, MultiLevelSurfaceGrid::SurfacePatch( 3.0, 0.1, 0.5, false ) );
    
    BinarySerialization serialization;
    EnvireBinaryItem bin_item;
    
    serialization.serializeBinaryItem(mls, bin_item);
    
    EnvironmentItem *new_item = serialization.unserializeBinaryItem(bin_item);
    BOOST_CHECK_EQUAL(new_item->getClassName(), mls->getClassName());
    
    MultiLevelSurfaceGrid* mls2 = dynamic_cast<MultiLevelSurfaceGrid*>(new_item);
    
    MultiLevelSurfaceGrid::iterator it = mls2->beginCell(0,0);
    BOOST_CHECK_EQUAL( it->mean, 2.0 );
    it++;
    BOOST_CHECK_EQUAL( (*it).mean, 1.0 );
    it++;
    BOOST_CHECK_EQUAL( it, mls2->endCell() );
    
    MultiLevelSurfaceGrid::iterator it2 = mls2->beginCell(2,1);
    BOOST_CHECK_EQUAL( it2->mean, 3.0 );
    it2++;
    BOOST_CHECK_EQUAL( it2, mls2->endCell() );
    
}

BOOST_AUTO_TEST_SUITE_END()