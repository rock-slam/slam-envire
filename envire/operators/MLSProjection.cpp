#include "MLSProjection.hpp"

using namespace envire;

const std::string MLSProjection::className = "envire::MLSProjection";

MLSProjection::MLSProjection()
    : gapSize( 1.0 ), thickness( 0.05 ), withUncertainty( true )
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
    for(MultiLevelSurfaceGrid::iterator it = grid->beginCell( m, n ); it != grid->endCell(); it++ )
    {
	// check for all surface patches in the cell, if the new point can be
	// part of that cell or if it needs to create a new cell
	MultiLevelSurfaceGrid::SurfacePatch &p( *it );
	const double delta_dev = sqrt( p.stdev * p.stdev + stdev * stdev );

	if( (p.mean - p.height - gapSize - delta_dev) < mean 
		&& (p.mean + gapSize + delta_dev) > mean )
	{
	    if( p.horizontal ) 
	    {
		if( (p.mean - p.height - thickness - delta_dev) < mean && 
			 (p.mean + thickness + delta_dev) > mean )
		{
		    // for horizontal patches, perform an update similar to the kalman
		    // update rule
		    double pvar = p.stdev * p.stdev;
		    double var = stdev * stdev;
		    double gain = pvar / (pvar + var);
		    p.mean = p.mean + gain * (mean - p.mean);
		    p.stdev = sqrt((1.0-gain)*pvar);
		}
		else
		{
		    // convert into a vertical patch element
		    p.mean += p.stdev;
		    p.height = 2 * p.stdev; 
		    p.horizontal = false;
		}
	    }

	    // no else here since we could have just been converted to a
	    // horizontal patch
	    if( !p.horizontal )
	    {
		if( mean > p.mean )
		{
		    p.height += ( mean - p.mean );
		    p.mean = mean;
		    p.stdev = stdev;
		}
		else
		{
		    p.height = std::max( p.height, p.mean - mean );
		}
	    }
	    return;
	}
    }

    MultiLevelSurfaceGrid::SurfacePatch p;
    p.mean = mean;
    p.stdev = stdev;
    p.height = 0;
    p.horizontal = true;

    grid->insertHead( m, n, p );
}

void MLSProjection::projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc )
{
    TransformWithUncertainty C_m2g = env->relativeTransformWithUncertainty( pc->getFrameNode(), grid->getFrameNode() );

    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    assert(points.size() == uncertainty.size());

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = uncertainty[i];
	PointWithUncertainty p = C_m2g * PointWithUncertainty( points[i], Eigen::Matrix3d::Identity() * p_var );

	const Eigen::Vector3d &mean( p.getPoint() );

	size_t x, y;
	if( grid->toGrid( mean.x(), mean.y(), x, y ) )
	{
	    const double stdev = sqrt(p.getCovariance()(2,2));
	    updateCell(grid, x, y, mean.z(), stdev);
	}
    }
}

void MLSProjection::projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc )
{
    Transform C_m2g = env->relativeTransform( pc->getFrameNode(), grid->getFrameNode() );

    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    assert(points.size() == uncertainty.size());

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = uncertainty[i];
	Point p = C_m2g * points[i];

	const Eigen::Vector3d &mean( p );

	size_t x, y;
	if( grid->toGrid( mean.x(), mean.y(), x, y ) )
	{
	    const double stdev = sqrt(p_var);
	    updateCell(grid, x, y, mean.z(), stdev);
	}
    }
}

bool MLSProjection::updateAll() 
{
    // TODO add checking of connections
    MultiLevelSurfaceGrid* grid = static_cast<envire::MultiLevelSurfaceGrid*>(*env->getOutputs(this).begin());

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	Pointcloud* mesh = dynamic_cast<envire::Pointcloud*>(*it);
	if( withUncertainty )
	    projectPointcloudWithUncertainty( grid, mesh );
	else
	    projectPointcloud( grid, mesh );
    }

    env->itemModified( grid );
    return true;
}

MLSProjection* MLSProjection::clone() const 
{
    return new MLSProjection( *this );
}

void MLSProjection::set( EnvironmentItem* other )
{
    MLSProjection* fn = dynamic_cast<MLSProjection*>( other ); 
    if( fn ) operator=( *fn );
}
