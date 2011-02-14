#include "MLSMap.hpp"

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSMap )

Eigen::AlignedBox<double, 2> MLSMap::getExtents() const 
{ 
    throw std::runtime_error("not implemented"); 
}

MLSMap::MLSMap(Serialization& so)
    : Map<2>( so )
{
}

MLSMap::MLSMap()
{
}

MLSMap::MLSMap(const MLSMap& other)
    : grids( other.grids ), active( other.active )
{
}

MLSMap& MLSMap::operator=(const MLSMap& other)
{
    grids = other.grids;
    active = other.active;

    return *this;
}

MultiLevelSurfaceGrid::SurfacePatch* 
    MLSMap::getPatch( const Point& p, const SurfacePatch& patch, double sigma_threshold )
{
    // go backwards in the list, and try to find the point in any of the grids
    for( std::vector<MultiLevelSurfaceGrid::Ptr>::reverse_iterator it = grids.rbegin(); it != grids.rend(); it++ )
    {
	MultiLevelSurfaceGrid::Ptr grid( *it );
	Transform C_m2g = env->relativeTransform( getFrameNode(), grid->getFrameNode() );
	MultiLevelSurfaceGrid::Position pos;
	if( grid->toGrid((C_m2g * p).start<2>(), pos) )
	{
	    return grid->get( pos, patch, sigma_threshold );
	}
    }
    return NULL;
}

void MLSMap::addGrid( MultiLevelSurfaceGrid::Ptr grid )
{
    grids.push_back( grid );
    active = grid;
}

void MLSMap::createGrid( const Transform& trans )
{
    // don't do anything if there is no template 
    if( !active )
	return;

    MultiLevelSurfaceGrid* grid_clone = active->cloneShallow(); 
    FrameNode* fn = new FrameNode( trans );
    env->addChild( active->getFrameNode(), fn );
    env->setFrameNode( grid_clone, fn );
}
