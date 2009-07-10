#include "ScanMeshing.hpp"
#include <stdexcept>

using namespace envire;

void ScanMeshing::addInput( LaserScan_Ptr scan ) 
{
    Operator::addInput(scan);
}

void ScanMeshing::addOutput( TriMesh_Ptr mesh )
{
    if( outputs.size() > 0 )
        throw std::runtime_error("ScanMeshing can only have one output.");

    Operator::addOutput(mesh);
}

void ScanMeshing::setMaxEdgeLength( float value ) 
{
    maxEdgeLength = value;
}

bool ScanMeshing::updateAll() 
{
    // TODO: add the generation of the mesh based on the 
    return false;
}
