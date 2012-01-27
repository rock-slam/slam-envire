#include "MLSProjection.hpp"
#include <set>
#include <Eigen/LU>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSProjection )

MLSProjection::MLSProjection()
    : withUncertainty( true )
{
}

MLSProjection::MLSProjection(Serialization& so)
{
    unserialize(so);
}

void MLSProjection::serialize(Serialization& so)
{
    Operator::serialize(so);
}

void MLSProjection::unserialize(Serialization& so)
{
    Operator::unserialize(so);
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
    TransformWithUncertainty C_m2g = env->relativeTransformWithUncertainty(
	    pc->getFrameNode(), grid->getFrameNode() );
    // create a new grid with the same dimensions in case the given grid is not
    // empty
    boost::intrusive_ptr<envire::MultiLevelSurfaceGrid> t_grid;
    if( !grid->empty() )
	t_grid = new MultiLevelSurfaceGrid( 
		grid->getWidth(), grid->getHeight(), grid->getScaleX(), grid->getScaleY(), grid->getOffsetX(), grid->getOffsetY() );
    else
	t_grid = grid;

    // make sure we are recording the cell positions in a set
    t_grid->initIndex();

    // store the updated positions in the vector, since we won't probably touch
    // so many items in the grid

    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    std::vector<Eigen::Vector3d> *color = NULL;
    if( pc->hasData( Pointcloud::VERTEX_COLOR ) )
    {
	color = &pc->getVertexData<Eigen::Vector3d>(Pointcloud::VERTEX_COLOR);
	t_grid->setHasCellColor( true );
	grid->setHasCellColor( true );
    }
    assert(points.size() == uncertainty.size());

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = uncertainty[i];
	Point p = C_m2g.getTransform() * points[i];

	const Eigen::Vector3d &mean( p );

	size_t xi, yi;
	if( t_grid->toGrid( mean.x(), mean.y(), xi, yi ) )
	{
	    const double stdev = sqrt(p_var);
	    MLSGrid::SurfacePatch patch( mean.z(), stdev );
	    if( color )
		patch.color = (*color)[i];
	    t_grid->updateCell(xi, yi, patch);
	}
    }

    Eigen::Affine3d C_g2m( C_m2g.getTransform().inverse( Eigen::Isometry ) );

    typedef MultiLevelSurfaceGrid::Position position;
    std::set<position> &cells = t_grid->getIndex()->cells;

    // go through all the cells that have been touched
    for(std::set<position>::iterator it = cells.begin(); it != cells.end(); it++)
    {
	const size_t xi = it->x;
	const size_t yi = it->y;

	for(MultiLevelSurfaceGrid::iterator cit = t_grid->beginCell(xi,yi); cit != t_grid->endCell(); cit++ )
	{
	    // get center of cell
	    double x, y;
	    t_grid->fromGrid( xi, yi, x, y );
	    Eigen::Vector3d cellcenter = C_g2m * Eigen::Vector3d(x, y, cit->mean);

	    // use the cells stdev for the point, this is not quite exact, but should do 
	    const double p_var = cit->stdev * cit->stdev;
	    PointWithUncertainty p = C_m2g * PointWithUncertainty( cellcenter, Eigen::Matrix3d::Zero() );

	    // write the transformed uncertainty back
	    cit->stdev = sqrt(p_var + p.getCovariance()(2,2));

	    // add the patch with the updated uncertainty into the target grid
	    // TODO: in case we are operating on the target grid already, 
	    // we may have to call some sort of merge, since through the updated
	    // variance we might have some patches that actually belong together.
	    if( t_grid != grid )
		grid->updateCell( xi, yi, *cit );
	}
    }
}

void MLSProjection::projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc )
{
    Transform C_m2g = env->relativeTransform( pc->getFrameNode(), grid->getFrameNode() );

    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    std::vector<Eigen::Vector3d> *color = NULL;
    if( pc->hasData( Pointcloud::VERTEX_COLOR ) )
    {
	color = &pc->getVertexData<Eigen::Vector3d>(Pointcloud::VERTEX_COLOR);
	grid->setHasCellColor( true );
    }
    assert(points.size() == uncertainty.size());

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = uncertainty[i];
	Point p = C_m2g * points[i];

	const Eigen::Vector3d &mean( p );

	size_t xi, yi;
	if( grid->toGrid( mean.x(), mean.y(), xi, yi ) )
	{
	    const double stdev = sqrt(p_var);
	    MLSGrid::SurfacePatch patch( mean.z(), stdev );
	    if( color )
		patch.color = (*color)[i];
	    grid->updateCell(xi, yi, patch);
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

