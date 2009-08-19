#include "envire/Core.hpp"
//#include "envire/LaserScan.hpp"
//#include "envire/TriMesh.hpp"
//#include "envire/ScanMeshing.hpp"

#define BOOST_TEST_MODULE EnvireTest 
#include <boost/test/included/unit_test.hpp>
   
/** Test scenario, for setting up an environment, and meshing a scan file
 * using the scanmeshing operator 
*/
#if 0
BOOST_AUTO_TEST_CASE( functional ) 
{
    // Set up the environment, which holds a pointer to the root FrameNode and
    // knows about all the layers
    envire::Environment_Ptr env = envire::Environment_Ptr( new envire::Environment("scene1") );

    // Create a laserscan layer from a scanfile and attach it to the rootnode
    // if the scan file has its own frame, the FrameNode will be generated and
    // used as the framenode for the LaserScan Layer.
    envire::LaserScan_Ptr scan = 
            envire::LaserScan::createFromScanFile("test/test.scan", env->getFrameNode());
    scan->setParent( env );

    // create a TriMesh Layer and attach it to the root Node.
    envire::TriMesh_Ptr mesh(new envire::TriMesh(env->getFrameNode(), "mesh1"));
    mesh->setParent( env );

    // set up a meshing operator on the output mesh. Add then an input
    // and parametrize the meshing operation. 
    envire::ScanMeshing_Ptr op( new envire::ScanMeshing());
    op->addInput(scan);
    op->addOutput(mesh);
    op->setMaxEdgeLength(0.5);

    op->updateAll();
}
#endif

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
};

class DummyLayer : public Layer 
{
public:
    Layer* clone() {return new DummyLayer(*this);};
};

class DummyCartesianMap : public CartesianMap 
{
public:
    CartesianMap* clone() {return new DummyCartesianMap(*this);};
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
    fn2->getTransform().rotation() = Eigen::Quaternionf( 0.0, 1.0, 0.0, 0.0 );
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
    env->addChild( fn3, env->getRootNode() );

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

// EOF
//
