#include "envire/core.hpp"

#define BOOST_TEST_MODULE EnvireTest 
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE( framenode ) 
{
    envire::Environment env;

    envire::LaserScan *scan = new envire::LaserScan(env.getRootNode());
    env.addLayer(scan);
    scan.parseScanFile("test/test.scan");

    envire::TriMesh *mesh = new envire::TriMesh(env.getRootNode());
    env.addLayer(mesh);

    envire::ScanMeshingOperator op( scan, mesh );
    op.setMaxEdgeLength(0.5);
    op.updateAll();
}

// EOF
//
