#include "GridAccess.hpp"

#include "Grids.hpp"

using namespace envire;

GridAccess::GridAccess(Environment* env)
    : env(env)
{
}

bool GridAccess::getElevation(Eigen::Vector3d& position)
{
    std::vector<ElevationGrid*> grids = env->getItems<ElevationGrid>();

    for(std::vector<ElevationGrid*>::iterator it = grids.begin();it != grids.end();it++)
    {
	ElevationGrid* grid = *it;
	FrameNode::TransformType t =
	    env->relativeTransform( 
		    grid->getFrameNode(),
		    env->getRootNode() );

	Eigen::Vector3d local( t * position );
	size_t x, y;
	if( grid->toGrid(local.x(), local.y(), x, y) )
	{
	    position.z() = grid->getGridData()[x][y];
	    return true;
	}
    }

    return false;
}
