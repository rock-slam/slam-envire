#include "envire/Core.hpp"
#include "envire/LaserScan.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"

#define BOOST_TEST_MODULE EnvireTest 
#include <boost/test/included/unit_test.hpp>

    
/** Test scenario, for setting up an environment, and meshing a scan file
 * using the scanmeshing operator 
 */
BOOST_AUTO_TEST_CASE( framenode ) 
{
    // Set up the environment, which holds a pointer to the root FrameNode and
    // knows about all the layers
    envire::Environment env;

    // Create a laserscan layer from a scanfile and attach it to the rootnode
    // if the scan file has its own frame, the FrameNode will be generated and
    // used as the framenode for the LaserScan Layer.
    envire::LaserScan_Ptr scan = 
            envire::LaserScan::createFromScanFile("test/test.scan", env.getRootNode());
    env.addLayer(scan);

    // set up a meshing operator on the output mesh. Add then an input
    // and parametrize the meshing operation. 
    envire::ScanMeshing_Ptr op( new envire::ScanMeshing());
    op->addInput( scan );
    op->setMaxEdgeLength(0.5);

    // create a TriMesh Layer and attach it to the root Node.
    envire::TriMesh_Ptr mesh(new envire::TriMesh(env.getRootNode(), op));
    op->addOutput(mesh);
    env.addLayer(mesh);

    op->updateAll();
}

BOOST_AUTO_TEST_CASE( environment ) 
{
    // set up the environment which holds the individual layers
    envire::Environment env;

    // populate the environment using a scene file
    env.loadSceneFile( "test/scene.esf" );
}
// EOF
//
