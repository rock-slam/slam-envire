#include "envire/Core.hpp"
#include "envire/LaserScan.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"

#define BOOST_TEST_MODULE EnvireTest 
#include <boost/test/included/unit_test.hpp>
   
using namespace envire;
using namespace std;

template<class T> bool contains(const std::list<T>& list, const T& element)
{
    return find( list.begin(), list.end(), element ) != list.end();
};

class DummyOperator : public Operator 
{
public:
    bool updateAll() {};
    void serialize(Serialization &) {};
};

class DummyLayer : public Layer 
{
public:
    Layer* clone() {return new DummyLayer(*this);};
    void serialize(Serialization &) {};
};

class DummyCartesianMap : public CartesianMap 
{
public:
    CartesianMap* clone() {return new DummyCartesianMap(*this);};
    void serialize(Serialization &) {};
};


BOOST_AUTO_TEST_CASE( environment )
{
    // set up an environment
    Environment* env = new Environment();

    // an environment should always have a root node 
    BOOST_CHECK( env->getRootNode() );

    // create some child framenodes
    FrameNode *fn1, *fn2, *fn3;
    fn1 = new FrameNode();
    fn1->getTransform().translation() += Eigen::Vector3f( 0.0, 0.0, 0.5 );
    fn2 = new FrameNode();
    fn2->getTransform().rotate(Eigen::Quaternionf( 0.0, 1.0, 0.0, 0.0 ));
    fn3 = new FrameNode();

    // attach explicitely
    env->attachItem( fn1 );
    BOOST_CHECK( fn1->isAttached() );
    BOOST_CHECK_EQUAL( fn1->getEnvironment(), env );
    env->addChild( env->getRootNode(), fn1 );
    BOOST_CHECK_EQUAL( env->getRootNode(), env->getParent(fn1) );
    BOOST_CHECK( contains(env->getChildren(env->getRootNode()),fn1) );

    // implicit attachment
    env->addChild( fn1, fn2 );
    BOOST_CHECK( fn2->isAttached() );
    
    // setup the rest of the framenodes
    env->addChild( env->getRootNode(), fn3 );

    // now do the same for layers
    Layer *l1, *l2, *l3;
    l1 = new DummyLayer();
    l2 = new DummyLayer();
    l3 = new DummyLayer();

    env->attachItem( l1 );
    BOOST_CHECK( l1->isAttached() );
    BOOST_CHECK_EQUAL( l1->getEnvironment(), env );

    env->addChild( l1, l2 );
    BOOST_CHECK_EQUAL( l1, env->getParent(l2) );
    BOOST_CHECK( contains(env->getChildren(l1),l2) );

    env->attachItem(l3);

    // CartesianMaps should work the same
    CartesianMap *m1, *m2;
    m1 = new DummyCartesianMap();
    m2 = new DummyCartesianMap();

    env->attachItem( m1 );
    env->attachItem( m2 );

    env->setFrameNode( m1, fn1 );
    env->setFrameNode( m2, fn1 );

    BOOST_CHECK_EQUAL( env->getFrameNode( m1 ), fn1 );
    BOOST_CHECK( contains(env->getMaps(fn1),m1) );
    BOOST_CHECK( contains(env->getMaps(fn1),m2) );
    
    // now to operators
    Operator *o1;
    o1 = new DummyOperator();
    env->attachItem( o1 );

    env->addInput( o1, l1 );
    env->addInput( o1, l2 );
    env->addOutput( o1, l3 );

    BOOST_CHECK( contains(env->getInputs(o1),l1) );
    BOOST_CHECK( contains(env->getOutputs(o1),l3) );

    delete env;
}

BOOST_AUTO_TEST_CASE( serialization )
{
    Serialization so;

    Environment* env = new Environment();

    // create some child framenodes
    FrameNode *fn1, *fn2, *fn3;
    fn1 = new FrameNode();
    fn1->getTransform().translation() += Eigen::Vector3f( 0.0, 0.0, 0.5 );
    fn2 = new FrameNode();
    fn2->getTransform().rotate(Eigen::Quaternionf( 0.0, 1.0, 0.0, 0.0 ));
    fn3 = new FrameNode();
    
    // attach explicitely
    env->attachItem( fn1 );
    env->addChild( env->getRootNode(), fn1 );
    env->addChild( fn1, fn2 );
    env->addChild( env->getRootNode(), fn3 );

    // TODO get cmake to somehow add an absolute path here
    std::string path("build/test");
    so.serialize(env, path);

    // now try to parse the thing again
    Environment* env2 = 
	so.unserialize( "build/test" );

    // TODO check that the structure is the same

    delete env;
    delete env2;
}

BOOST_AUTO_TEST_CASE( functional ) 
{
    Environment* env = new Environment(); 

    LaserScan* scan = LaserScan::importScanFile("test/test.scan", env->getRootNode() );

    // create a TriMesh Layer and attach it to the root Node.
    TriMesh* mesh = new TriMesh();
    env->attachItem( mesh );

    // set up a meshing operator on the output mesh. Add then an input
    // and parametrize the meshing operation. 
    ScanMeshing* mop = new ScanMeshing();
    env->attachItem( mop );

    mop->setMaxEdgeLength(0.5);

    mop->addInput(scan);
    mop->addOutput(mesh);

    mop->updateAll();

    Serialization so;
    so.serialize(env, "build/test");
}

// EOF
//
