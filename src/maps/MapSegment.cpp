#include "MapSegment.hpp"
#include <envire/core/Serialization.hpp>
#include <envire/tools/ExpectationMaximization.hpp>

#include <limits>

using namespace envire;
using namespace std;

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

void MapSegment::addPart( const base::Affine3d& pose, CartesianMap* map, double weight )
{
    Part p = { map, base::Pose( pose ), weight };
    parts.push_back( p );
}

void MapSegment::update()
{
    // run the full ExpectationMinimization algorithm 
    // on the parts already stored
    vector<base::Vector6d> em_pars;
    vector<double> em_weights;
    envire::ExpectationMaximization<GMM> em;
    for( size_t i=0; i<parts.size(); i++ )
    {
	em_pars.push_back( parts[i].pose.toVector6d() );
	em_weights.push_back( parts[i].weight );
    }
    em.initialize( 5, em_pars, em_weights );
    em.run( 1e-5, 10 );

    // and store the resulting gmm
    gmm.params.swap( em.gmm.params );
}

TransformWithUncertainty MapSegment::getTransform() const
{
    // for now, just return the first transform of the GMM model
    // later we should also take into account the "mixture" part
    // of the model and do some permutations perhaps 
    
    // take the largest of the gaussians and use it as the new centroid
    int max_idx = -1;
    int max_weight = 0;
    for( size_t i=0; i < gmm.params.size(); i++ )
    {
	if( gmm.params[i].weight > max_weight )
	{
	    max_idx = i;
	    max_weight = gmm.params[i].weight;
	}
    }
    if( max_idx >=0 )
    {
	return TransformWithUncertainty(
		base::Pose( gmm.params[max_idx].dist.mean ).toTransform(),
		gmm.params[max_idx].dist.cov );
    }

    throw std::runtime_error("MapSegment does not have a pose distribution.");
}

CartesianMap* MapSegment::getMapForPose( const base::Affine3d& pose ) const
{
    // find the pose in the parts, which has the smallest distance to 
    // the provided pose
    // For now we do an extensive search, instead of e.g. using a kd-tree
    // since this function shouldn't be called too often

    base::Vector6d pose6d = base::Pose(pose).toVector6d();

    CartesianMap* best_map = NULL;
    double best_dist = numeric_limits<double>::infinity();
    for( size_t i=0; i < parts.size(); i++ )
    {
	// for now take the norm, this may need some different
	// weighting of distance and angular values
	double dist = (parts[i].pose.toVector6d() - pose6d).norm();
	if( dist < best_dist )
	{
	    best_map = parts[i].map.get();
	    best_dist = dist;
	}
    }

    return best_map;
}

