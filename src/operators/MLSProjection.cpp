#include "MLSProjection.hpp"
#include <set>
#include <Eigen/LU>

#include <envire/tools/BresenhamLine.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSProjection )

MLSProjection::MLSProjection()
    : withUncertainty( true ), m_negativeInformation( false ), defaultUncertainty( 0.01 ), use_boundary_box(false)
{
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

void MLSProjection::setAreaOfInterest(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
    boundary_box.setEmpty();
    boundary_box.extend(Eigen::Vector3d(min_x, min_y, min_z));
    boundary_box.extend(Eigen::Vector3d(max_x, max_y, max_z));
    use_boundary_box = true;
}

void MLSProjection::unsetAreaOfInterest()
{
    boundary_box.setEmpty();
    use_boundary_box = false;
}

void MLSProjection::projectPointcloudWithUncertainty( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc )
{
    // create a new grid with the same dimensions in case the given grid is not
    // empty
    boost::intrusive_ptr<envire::MultiLevelSurfaceGrid> t_grid;
    if( !grid->empty() )
    {
	t_grid = new MultiLevelSurfaceGrid( 
		grid->getWidth(), grid->getHeight(), grid->getScaleX(), grid->getScaleY(), grid->getOffsetX(), grid->getOffsetY() );
	t_grid->getConfig() = grid->getConfig();
    }
    else
	t_grid = grid;

    // make sure we are recording the cell positions in a set
    t_grid->initIndex();
    projectPointcloud( t_grid.get(), pc );

    Eigen::Affine3d C_g2m( C_m2g.getTransform().inverse( Eigen::Isometry ) );

    // get the origin of the poincloud as a map cell
    Eigen::Vector3d origin_m = C_m2g.getTransform() * pc->getSensorOrigin().translation();
    GridBase::Position origin;
    if( !grid->toGrid( origin_m.head<2>(), origin ) )
	if( m_negativeInformation )
	    throw std::runtime_error( "origin of pointcloud needs to be within grid." );

    // go through all the cells that have been touched
    typedef MultiLevelSurfaceGrid::Position position;
    const std::set<position> &cells = t_grid->getIndex()->cells;

    for(std::set<position>::iterator it = cells.begin(); it != cells.end(); it++)
    {
	const size_t xi = it->x;
	const size_t yi = it->y;

	for(MultiLevelSurfaceGrid::iterator cit = t_grid->beginCell(xi,yi); cit != t_grid->endCell(); cit++ )
	{
	    if( withUncertainty )
	    {
		// this method also incorporates the uncertainty in the transform
		// instead of just the uncertainty from the pointcloud items
		// this is done by transforming the pointcloud to temporary mls
		// grid first, and then "smearing" the uncertainty over the already
		// indexed cells. This is mainly for performance issues, since calculating
		// the transform for each input point is much more costly.

		// get center of cell
		double x, y;
		t_grid->fromGrid( xi, yi, x, y );
		Eigen::Vector3d cellcenter = C_g2m * Eigen::Vector3d(x, y, cit->mean);

		// use the cells stdev for the point, this is not quite exact, but should do 
		const double p_var = cit->stdev * cit->stdev;
		PointWithUncertainty p = C_m2g * PointWithUncertainty( cellcenter, Eigen::Matrix3d::Zero() );

		// write the transformed uncertainty back
		cit->stdev = sqrt(p_var + p.getCovariance()(2,2));
	    }

	    // add the patch with the updated uncertainty into the target grid
	    // TODO: in case we are operating on the target grid already, 
	    // we may have to call some sort of merge, since through the updated
	    // variance we might have some patches that actually belong together.
	    if( t_grid != grid )
		grid->updateCell( xi, yi, *cit );

	    if( m_negativeInformation )
	    {
		// in order to handle negative information (e.g. knowledge that
		// a cell is free), we use bresenhams line algorithm to find the cells
		// from each known cells to the origin. For each of these cells, 
		// we add absence information
		
		// this is the distance on the x/y plane from origin to 
		// the current cell
		const double plane_dist = (grid->fromGrid( origin ) - grid->fromGrid( *it )).norm();

		// this is the height difference between the origin and the cell
		const double plane_z = origin_m.z() - cit->mean;

		// this is the maximum height of the negative cell,
		// which joins the height of the cell, and how high it
		// is perceived from the origin point of view
		double height = 0; //cit->height;
		if( plane_dist * plane_z != 0 )
		    height += grid->getScaleX() / plane_dist * plane_z;

		std::vector<GridBase::Position> line_cells;
		lineBresenham( *it, origin, line_cells );

		int xdiff = it->x - origin.x;
		int ydiff = it->y - origin.y;
		bool xdir = abs( xdiff ) >= abs( ydiff );

		for( size_t li = 0; li < line_cells.size(); li++ )
		{
		    double factor = 1.0;
		    if( xdir && xdiff )
			factor = (int)(line_cells[li].x - origin.x) / (double)xdiff;
		    else if( ydiff )
			factor = (int)(line_cells[li].y - origin.y) / (double)ydiff;

		    const double p_height = fabs((cit->height + height) * factor);
		    const double z_mean = cit->mean + height * factor + plane_z * (1.0 - factor);
		    const double z_stdev = cit->stdev * factor; 

		    MLSGrid::SurfacePatch np( z_mean, z_stdev, p_height, MLSGrid::SurfacePatch::NEGATIVE );

		    // for now don't put anything into the cell with the
		    // positive information. This could be changed later
		    // for partial information
		    if( factor < 1.0 )
			grid->updateCell( line_cells[li], np );
		}
	    }
	}
    }
}

void MLSProjection::projectPointcloud( envire::MultiLevelSurfaceGrid* grid, envire::Pointcloud* pc )
{
    // note: the grid might actually be a local copy and not attached to an
    // environment
    std::vector<Eigen::Vector3d>& points(pc->vertices);
    std::vector<double>& uncertainty(pc->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    std::vector<Eigen::Vector3d> *color = NULL;
    if( pc->hasData( Pointcloud::VERTEX_COLOR ) )
    {
	color = &pc->getVertexData<Eigen::Vector3d>(Pointcloud::VERTEX_COLOR);
	assert( color->size() == points.size() );
	grid->setHasCellColor( true );
    }
    bool hasUncertainty = points.size() == uncertainty.size();

    for(size_t i=0;i<points.size();i++)
    {
	const double p_var = hasUncertainty? uncertainty[i] : defaultUncertainty;
	Point p = C_m2g.getTransform() * points[i];

	const Eigen::Vector3d &mean( p );

        if(use_boundary_box && !boundary_box.contains(mean))
            continue;

        // create patch to update
        MLSGrid::SurfacePatch patch( mean.z(), sqrt(p_var) );
        if( color )
            patch.setColor( (*color)[i] );

        // and use the update method of the mls to determine
        // which cell and update model to use
        grid->update( mean.head<2>(), patch );
    }
}

bool MLSProjection::updateAll() 
{
    std::list<Layer*> outputs = env->getOutputs(this);
    if(outputs.empty())
        return false;
    
    MultiLevelSurfaceGrid* grid = dynamic_cast<envire::MultiLevelSurfaceGrid*>(*(outputs.begin()));
    if(!grid)
        return false;
    
    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	Pointcloud* mesh = dynamic_cast<envire::Pointcloud*>(*it);

	C_m2g = env->relativeTransformWithUncertainty(
	    mesh->getFrameNode(), grid->getFrameNode() );

	if( withUncertainty || m_negativeInformation )
	    projectPointcloudWithUncertainty( grid, mesh );
	else
	    projectPointcloud( grid, mesh );
    }

    env->itemModified( grid );
    return true;
}

