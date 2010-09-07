#include "MLSProjection.hpp"

using namespace envire;

const std::string MLSProjection::className = "envire::MLSProjection";

MLSProjection::MLSProjection()
    : gapSize( 1.0 ), thickness( 0.1 )
{
}

MLSProjection::MLSProjection(Serialization& so)
    : Operator(so)
{
    so.setClassName(className);
}

void MLSProjection::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.setClassName(className);
}


void MLSProjection::addInput( Pointcloud* mesh ) 
{
    Operator::addInput(mesh);
}

void MLSProjection::addOutput( MultiLevelSurfaceGrid* grid )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("MLSProjection can only have one output.");

    Operator::addOutput(grid);
}

void MLSProjection::updateCell(MultiLevelSurfaceGrid* grid, size_t m, size_t n, double mean, double stdev )
{
    MultiLevelSurfaceGrid::iterator it = grid->beginCell( m, n );
    while( it.isValid() )
    {
	MultiLevelSurfaceGrid::SurfacePatch &p( *it );
	if( (p.mean - p.height - gapSize) < mean && (p.mean + gapSize) > mean )
	{
	    if( p.horizontal && 
		    ((p.mean - p.height - thickness ) < mean && 
		     (p.mean + thickness) > mean ) )
	    {
		double pvar = p.stdev * p.stdev;
		double var = stdev * stdev;
		double gain = pvar / (pvar + var);
		p.mean = p.mean + gain * (mean - p.mean);
		p.stdev = sqrt((1.0-gain)*pvar);
	    }
	    else
	    {
		p.horizontal = false;
	    }

	    if( !p.horizontal )
	    {
		if( mean > p.mean )
		{
		    p.mean = mean;
		    p.stdev = stdev;
		    p.height += ( mean - p.mean );
		}
		else
		{
		    p.height = std::max( p.height, p.mean - mean );
		}
	    }
	    return;
	}
	it++;
    }

    MultiLevelSurfaceGrid::SurfacePatch p;
    p.mean = mean;
    p.stdev = stdev;
    p.height = 0;
    p.horizontal = true;

    grid->insertHead( m, n, p );
}

bool MLSProjection::updateAll() 
{
    // TODO add checking of connections
    MultiLevelSurfaceGrid* grid = static_cast<envire::MultiLevelSurfaceGrid*>(*env->getOutputs(this).begin());

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	Pointcloud* mesh = dynamic_cast<envire::Pointcloud*>(*it);

	FrameNode::TransformType C_m2g = env->relativeTransform( mesh->getFrameNode(), grid->getFrameNode() );

	std::vector<Eigen::Vector3d>& points(mesh->vertices);
	
	for(size_t i=0;i<points.size();i++)
	{
	    Eigen::Vector3d p = env->getRootNode()->getTransform() * C_m2g * points[i];

	    size_t x, y;
	    if( grid->toGrid( p.x(), p.y(), x, y ) )
	    {
		const double stdev = 0.02;
		updateCell(grid, x, y, p.z(), stdev);
	    }
	}
    }

    return true;
}
