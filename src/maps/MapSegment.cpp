#include "MapSegment.hpp"
#include <envire/core/Serialization.hpp>

using namespace envire;
ENVIRONMENT_ITEM_DEF( MapSegment )

envire::Map<3>::Extents MapSegment::getExtents() const
{
    // combine the extents of the children of this map
    Eigen::AlignedBox<double, 3> extents;

    for( std::vector<Part>::const_iterator it = parts.begin(); it != parts.end(); it++ )
    {
	const envire::CartesianMap *map = it->map.get();
	const Map<2>* map2 = dynamic_cast<const Map<2>*>( map );
	const Map<3>* map3 = dynamic_cast<const Map<3>*>( map );
	Eigen::Affine3d g2m = map->getFrameNode()->relativeTransform( this->getFrameNode() );

	// currently this assumes that child grids are not rotated
	// TODO handle rotated child grids here
	if( map3 )
	{
	    extents.extend( map3->getExtents().translate( g2m.translation() ) );
	}
	else if( map2 )
	{
	    Eigen::AlignedBox<double, 3> e3;
	    e3.min().head<2>() = map2->getExtents().min();
	    e3.max().head<2>() = map2->getExtents().max();
	    extents.extend( e3.translate( g2m.translation() ) );
	}
    }

    return extents;
}

void MapSegment::addPart( const base::Affine3d& pose, CartesianMap* map )
{
    Part p = { map, pose };
    parts.push_back( p );
}

void MapSegment::update()
{
    // run the full ExpectationMinimization algorithm 
    // on the parts already stored
}

TransformWithUncertainty MapSegment::getTransform() const
{
    // for now, just return the first transform of the GMM model
    // later we should also take into account the "mixture" part
    // of the model and do some permutations perhaps 
}

CartesianMap* MapSegment::getMapForPose( const base::Affine3d& pose ) const
{
    // find the pose in the parts, which has the smallest distance to 
    // the provided pose


}

