#include "Projection.hpp"
#include <stdexcept>
#include "boost/multi_array.hpp"

using namespace envire;
using namespace std;

const std::string Projection::className = "envire::Projection";

Projection::Projection()
{
}

Projection::Projection(Serialization& so)
    : Operator(so)
{
    so.setClassName(className);
}

void Projection::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.setClassName(className);
}


void Projection::addInput( TriMesh* mesh ) 
{
    Operator::addInput(mesh);
}

void Projection::addOutput( Grid* grid )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("Projection can only have one output.");

    Operator::addOutput(grid);
}

bool Projection::updateAll() 
{
    // TODO add checking of connections
    Grid* grid = static_cast<envire::Grid*>(*env->getOutputs(this).begin());

    boost::multi_array<double,2>& elevation(grid->getData<double>(Grid::ELEVATION));

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	TriMesh* mesh = dynamic_cast<envire::TriMesh*>(*it);

	FrameNode::TransformType C_m2g = env->relativeTransform( mesh->getFrameNode(), grid->getFrameNode() );

	std::vector<Eigen::Vector3d>& points(mesh->vertices);
	
	for(int i=0;i<points.size();i++)
	{
	    Eigen::Vector3d p = C_m2g * points[i];


	    size_t x, y;
	    if( grid->toGrid( p.x(), p.y(), x, y ) )
	    {
		elevation[x][y] = std::max( elevation[x][y], p.z() );
	    }
	}
    }
}

