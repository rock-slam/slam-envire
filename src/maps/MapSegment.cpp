#include "MapSegment.hpp"
#include <envire/core/Serialization.hpp>
#include <envire/tools/ExpectationMaximization.hpp>

#include <limits>
#include <numeric/Stats.hpp>

using namespace envire;
using namespace std;

ENVIRONMENT_ITEM_DEF( MapSegment )

MapSegment::MapSegment()
    : m_zVar(0)
{
}

envire::Map<3>::Extents MapSegment::getExtents() const
{
    // combine the extents of the children of this map
    Eigen::AlignedBox<double, 3> extents;

    for( std::vector<Part>::const_iterator it = parts.begin(); it != parts.end(); it++ )
    {
	const envire::CartesianMap *map = it->map.get();
	const Map<2>* map2 = dynamic_cast<const Map<2>*>( map );
	const Map<3>* map3 = dynamic_cast<const Map<3>*>( map );
	assert( this->isAttached() );
	assert( map->isAttached() );
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

void MapSegment::addPart( const base::Affine3d& pose, CartesianMap* map, double weight, double zVar )
{
    Part p = { map, base::Pose( pose ), weight };
    parts.push_back( p );
    m_zVar += weight * zVar;
}

void MapSegment::update()
{
    // simplified version:
    // instead of using a full GMM model,
    // for now we just use a single gaussian.

    base::Stats< base::Vector6d > stats;
    for( size_t i=0; i<parts.size(); i++ )
    {
	stats.update(
	    parts[i].pose.toVector6d(),
	    parts[i].weight );
    }
    double zVar = m_zVar / stats.sumWeights();

    // HACK !!!
    // augment the covariance with some uncertainties
    double 
	xrotVar = pow((1.0/180)*M_PI, 2),
	yrotVar = pow((1.0/180)*M_PI, 2);

    base::Vector6d diag;
    diag << xrotVar, yrotVar, 0, 0, 0, zVar;

    // copy to gmm
    gmm.params.clear();
    envire::Gaussian<double,6> gaussian( 
	    stats.mean(), 
	    base::Matrix6d(stats.var()) + base::Matrix6d(diag.asDiagonal()) );
    gmm.params.push_back( GMM::Parameter( 1.0, gaussian ) );
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

CartesianMap* MapSegment::getMapForPose( const base::Affine3d& pose, base::Affine3d& map_pose, size_t &traj_size ) const
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
	
	// TODO need to ignore the z (and probably pitch and roll) part!
	const double dist = (parts[i].pose.toVector6d() - pose6d).norm();
	if( dist < best_dist )
	{
	    best_map = parts[i].map.get();
	    map_pose = parts[i].pose.toTransform();
	    best_dist = dist;
	    if( trajectories.size() > i )
		traj_size = trajectories[i].size();
	}
    }

    return best_map;
}

CartesianMap* MapSegment::getBestMap() const
{
    CartesianMap* best_map = NULL;
    double best_weight = 0;
    for( size_t i=0; i < parts.size(); i++ )
    {
	const double weight = parts[i].weight;
	if( weight > best_weight )
	{
	    best_map = parts[i].map.get();
	    best_weight = weight;
	}
    }

    return best_map;
}
