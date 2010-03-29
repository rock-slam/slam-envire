#include "Projection.hpp"
#include <stdexcept>
#include <stdint.h>
#include <limits>
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
    updateElevationMap();
    updateTraversibilityMap();
}

bool Projection::updateElevationMap()
{
    // TODO add checking of connections
    Grid* grid = static_cast<envire::Grid*>(*env->getOutputs(this).begin());

    boost::multi_array<double,2>& elv_min(grid->getGridData<double>(Grid::ELEVATION_MIN));
    boost::multi_array<double,2>& elv_max(grid->getGridData<double>(Grid::ELEVATION_MAX));

    // fill the elevation map
    std::fill(elv_min.data(), elv_min.data() + elv_min.num_elements(), std::numeric_limits<double>::infinity());
    std::fill(elv_max.data(), elv_max.data() + elv_max.num_elements(), -std::numeric_limits<double>::infinity());

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	TriMesh* mesh = dynamic_cast<envire::TriMesh*>(*it);

	FrameNode::TransformType C_m2g = env->relativeTransform( mesh->getFrameNode(), grid->getFrameNode() );

	std::vector<Eigen::Vector3d>& points(mesh->vertices);
	
	for(int i=0;i<points.size();i++)
	{
	    Eigen::Vector3d p = env->getRootNode()->getTransform() * C_m2g * points[i];

	    size_t x, y;
	    if( grid->toGrid( p.x(), p.y(), x, y ) )
	    {
		elv_max[x][y] = std::max( elv_max[x][y], p.z() );
		elv_min[x][y] = std::min( elv_min[x][y], p.z() );
	    }
	}
    }

    return true;
}

bool Projection::updateTraversibilityMap()
{
    // TODO add checking of connections
    Grid* grid = static_cast<envire::Grid*>(*env->getOutputs(this).begin());

    if( !grid->hasData(Grid::ELEVATION_MIN) || !grid->hasData(Grid::ELEVATION_MAX) )
    {
	std::cout << "needs DEM map to calculate traversibility map." << std::endl;
	return false;
    }

    boost::multi_array<double,2>& elv_min(grid->getGridData<double>(Grid::ELEVATION_MIN));
    boost::multi_array<double,2>& elv_max(grid->getGridData<double>(Grid::ELEVATION_MAX));

    // compute the traversability map now...
    // this should most probably be done somewhere else, but is here for convenience now.
    boost::multi_array<uint8_t,2>& trav(grid->getGridData<uint8_t>(Grid::TRAVERSABILITY));
    std::fill(trav.data(), trav.data() + trav.num_elements(), 255);

    size_t width = elv_min.shape()[0]; 
    size_t height = elv_min.shape()[1]; 

    double scalex = grid->getScaleX();
    double scaley = grid->getScaleY();
    
    for(int x=1;x<(width-1);x++)
    {
	for(int y=1;y<(height-1);y++)
	{
	    double max_grad = -std::numeric_limits<double>::infinity();

	    for(int i=0;i<3;i++)
	    {
		// get the 4 neighbours
		int m = (i<2)*(i*2-1);
		int n = (i>=2)*((i-2)*2-1);
		double scale = (i<2)?scalex:scaley;

		double gradient =
		   std::max( 
			  std::abs(elv_min[x][y] - elv_max[x+m][y+n])/scale,
			  std::abs(elv_max[x][y] - elv_min[x+m][y+n])/scale );

		max_grad = std::max( max_grad, gradient );
	    }

	    double angle = atan( max_grad );

	    trav[x][y] = ((uint8_t)(angle/(M_PI/2.0)*6.0));
	}
    }

    return true;
}

