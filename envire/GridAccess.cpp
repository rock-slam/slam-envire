#include "GridAccess.hpp"

#include "Grids.hpp"

using namespace envire;

struct GridAccessImpl
{
    GridAccessImpl() : grid(NULL) {};

    std::vector<ElevationGrid*> grids;

    ElevationGrid* grid;
    ElevationGrid::ArrayType* gridData;
    FrameNode::TransformType t;

    bool evalGridPoint(Eigen::Vector3d& position)
    {
	Eigen::Vector3d local( t * position );
	size_t x, y;
	if( grid->toGrid(local.x(), local.y(), x, y) )
	{
	    position.z() = (*gridData)[y][x];
	    return true;
	}
	return false;
    }
};


GridAccess::GridAccess(Environment* env)
    : env(env), impl( new GridAccessImpl )
{
}

GridAccess::~GridAccess()
{
    delete impl;
}

bool GridAccess::getElevation(Eigen::Vector3d& position)
{
    // to make this fast, we store the last grid and transform
    // and see try that one first on the next call
    if( impl->grid )
    {
	return impl->evalGridPoint( position );
//	if( impl->evalGridPoint( impl->grid, impl->t, position ) )
//	    return true;
    }
    std::cout << "non cached!" << std::endl;

    if( impl->grids.size() == 0 )
	impl->grids = env->getItems<ElevationGrid>();

    for(std::vector<ElevationGrid*>::iterator it = impl->grids.begin();it != impl->grids.end();it++)
    {
	ElevationGrid* grid = *it;
	FrameNode::TransformType t =
	    env->relativeTransform( 
		    env->getRootNode(),
		    grid->getFrameNode() );

	impl->grid = grid;
	impl->gridData = &grid->getGridData();
	impl->t = t;

	if( impl->evalGridPoint( position ) )
	    return true;
    }

    return false;
}
