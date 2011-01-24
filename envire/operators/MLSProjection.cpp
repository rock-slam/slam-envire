#include "MLSProjection.hpp"

using namespace envire;

const std::string MLSProjection::className = "envire::MLSProjection";

MLSProjection::MLSProjection()
    : withUncertainty( true )
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
	    grid->updateCell(x, y, mean.z(), stdev);
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
	    grid->updateCell(x, y, mean.z(), stdev);
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
