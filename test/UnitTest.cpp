#include "envire/Core.hpp"
#include "envire/LaserScan.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"

#define BOOST_TEST_MODULE EnvireTest 
#include <boost/test/included/unit_test.hpp>

   
/** Test scenario, for setting up an environment, and meshing a scan file
 * using the scanmeshing operator 
 */
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
    mesh->setParent( mesh );

    // set up a meshing operator on the output mesh. Add then an input
    // and parametrize the meshing operation. 
    envire::ScanMeshing_Ptr op( new envire::ScanMeshing());
    op->addInput(scan);
    op->addOutput(mesh);
    op->setMaxEdgeLength(0.5);

    op->updateAll();
}

BOOST_AUTO_TEST_CASE( structure ) 
{
    envire::Environment_Ptr env = envire::Environment_Ptr(new envire::Environment("scene1"));
    BOOST_CHECK_EQUAL( "scene1", env->getID() );
    BOOST_CHECK( env->getFrameNode() );
   
    // create a child node of the root frame node
    envire::FrameNode_Ptr node1 = envire::FrameNode_Ptr(new envire::FrameNode());
    node1->setParent( env->getFrameNode() );
    BOOST_CHECK_EQUAL( node1->getParent(), env->getFrameNode() );
    BOOST_CHECK( env->getFrameNode()->beginNodes() !=  env->getFrameNode()->endNodes() );
    BOOST_CHECK_EQUAL( node1, *env->getFrameNode()->beginNodes() );
    BOOST_CHECK_EQUAL( std::distance( env->getFrameNode()->beginNodes(), env->getFrameNode()->endNodes() ), 1 );

    // create a new layer
    envire::TriMesh_Ptr map1 = envire::TriMesh_Ptr(new envire::TriMesh("mesh1"));
    BOOST_CHECK_EQUAL( "mesh1", map1->getID() );
    BOOST_CHECK_EQUAL( map1->getFrameNode(), envire::FrameNode_Ptr() );
    
    // attach layer to environment
    map1->setParent( env );
    BOOST_CHECK_EQUAL( map1->getParent(), env );
    BOOST_CHECK_EQUAL( *env->beginLayers(), map1 );
    BOOST_CHECK_EQUAL( std::distance( env->beginLayers(), env->endLayers() ), 1 );
    
    // associate layer with FrameNode
    map1->setFrameNode( node1 );
    BOOST_CHECK_EQUAL( map1->getFrameNode(), node1 );
    BOOST_CHECK_EQUAL( *node1->beginMaps(), map1 );
    BOOST_CHECK_EQUAL( std::distance( node1->beginMaps(), node1->endMaps() ), 1 );

    // reassociation to different framenode
    map1->setFrameNode( env->getFrameNode() );
    BOOST_CHECK_EQUAL( map1->getFrameNode(), env->getFrameNode() );
    BOOST_CHECK_EQUAL( std::distance( node1->beginMaps(), node1->endMaps() ), 0 );
    BOOST_CHECK_EQUAL( std::distance( env->getFrameNode()->beginMaps(), env->getFrameNode()->endMaps() ), 1 );

    // create a new layer
    envire::LaserScan_Ptr map2 = envire::LaserScan_Ptr(new envire::LaserScan("scan1"));
    BOOST_CHECK_EQUAL( "scan1", map2->getID() );
    map2->setParent( env );
    BOOST_CHECK_EQUAL( std::distance( env->beginLayers(), env->endLayers() ), 2 );

    // create operator
    BOOST_CHECK( !map2->isGenerated() );
    BOOST_CHECK( !map1->isGenerated() );
    envire::ScanMeshing_Ptr op1 = envire::ScanMeshing_Ptr(new envire::ScanMeshing());
    op1->addInput( map2 );
    op1->addOutput( map1 );
    BOOST_CHECK( map1->isGenerated() );
    BOOST_CHECK_EQUAL( map1->getGenerator(), op1 );

    map1->detachFromOperator();
    BOOST_CHECK( !map2->isGenerated() );
    BOOST_CHECK( !map1->isGenerated() );
}
// EOF
//
