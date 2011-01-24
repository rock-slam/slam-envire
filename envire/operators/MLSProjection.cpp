#include "MLSProjection.hpp"
#include <set>

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
    // create a new grid with the same dimensions
    boost::intrusive_ptr<envire::MultiLevelSurfaceGrid> 
	t_grid( new MultiLevelSurfaceGrid( grid->getWidth(), grid->getHeight(), grid->getScaleX(), grid->getScaleY() ) );
    // store the updated positions in the vector, since we won't probably touch
    // so many items in the grid
    typedef std::pair<size_t,size_t> position;
    std::set<position> pos_vector;

    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    assert(points.size() == uncertainty.size());

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = uncertainty[i];
	Point p = C_m2g.getTransform() * points[i];

	const Eigen::Vector3d &mean( p );

	size_t m, n;
	if( t_grid->toGrid( mean.x(), mean.y(), m, n ) )
	{
	    const double stdev = sqrt(p_var);
	    t_grid->updateCell(m, n, mean.z(), stdev);
	    pos_vector.insert( std::make_pair( m, n ) );
	}
    }

    // go through all the cells that have been touched
    for(std::set<position>::iterator it = pos_vector.begin(); it != pos_vector.end(); it++)
    {
	size_t m = it->first;
	size_t n = it->second;

	for(MultiLevelSurfaceGrid::iterator cit = t_grid->beginCell(m,n); cit != t_grid->endCell(); cit++ )
	{
	    // get center of cell
	    double x, y;
	    t_grid->fromGrid( m, n, x, y );
	    Eigen::Vector3d cellcenter( x, y, cit->mean );

	    // use the cells stdev for the point, this is not quite exact, but should do 
	    const double p_var = cit->stdev * cit->stdev;
	    PointWithUncertainty p = C_m2g * PointWithUncertainty( cellcenter, Eigen::Matrix3d::Zero() );

	    // write the transformed uncertainty back
	    cit->stdev = sqrt(p_var + p.getCovariance()(2,2));

	    // add the patch with the updated uncertainty into the target grid
	    grid->updateCell( m, n, *cit );
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
