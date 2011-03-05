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

    if( isAttached() )
    {
	std::list<Layer*> children = env->getChildren( this );
	for( std::list<Layer*>::iterator it = children.begin(); it != children.end(); it++ )
	{
	    env->removeChild( this, *it );
	}

	typedef std::vector<MultiLevelSurfaceGrid::Ptr>::iterator iterator;
	for( iterator it = grids.begin(); it != grids.end(); it++ )
	{
	    env->addChild( this, (*it).get() );
	}
    }

    return *this;
}

bool MLSMap::getPatch( const Point& p, SurfacePatch& patch, double sigma_threshold )
{
    // go backwards in the list, and try to find the point in any of the grids
    for( std::vector<MultiLevelSurfaceGrid::Ptr>::reverse_iterator it = grids.rbegin(); it != grids.rend(); it++ )
    {
	MultiLevelSurfaceGrid::Ptr grid( *it );
	Transform C_m2g = env->relativeTransform( getFrameNode(), grid->getFrameNode() );
	MultiLevelSurfaceGrid::Position pos;
	if( grid->toGrid((C_m2g * p).start<2>(), pos) )
	{
	    // offset the z-coordinate which is given in map to grid 
	    MultiLevelSurfaceGrid::SurfacePatch probe( patch );
	    probe.mean += C_m2g.translation().z();
	    
	    MultiLevelSurfaceGrid::SurfacePatch* res = 
		grid->get( pos, probe, sigma_threshold );

	    if( res )
	    {
		patch = *res;
		// offset the z-coordinate, so that it is expressed in terms
		// of the map and not the grid
		patch.mean -= C_m2g.translation().z();
		return true;
	    }
	}
    }
    return false;
}

void MLSMap::addGrid( MultiLevelSurfaceGrid::Ptr grid )
{
    env->addChild( this, grid.get() );

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

    addGrid( grid_clone );
}

MLSMap* MLSMap::cloneDeep()
{
    MLSMap* res = clone();
    // copy the layer structure as well
    if( env )
    {
	// create a copy of the currently active map
	// and reference the others
	MultiLevelSurfaceGrid* active_clone = active->clone();
	res->grids.back() = active_clone;
	res->active = active_clone;

	env->setFrameNode( active_clone, active->getFrameNode() );
	
	env->attachItem( res );

	typedef std::vector<MultiLevelSurfaceGrid::Ptr>::iterator iterator;
	for( iterator it = res->grids.begin(); it != res->grids.end(); it++ )
	{
	    env->addChild( res, (*it).get() );
	}
    }
    return res;
}

