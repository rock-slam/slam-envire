#include "tools/GridAccess.hpp"

#include "maps/Grids.hpp"
#include "maps/Pointcloud.hpp"
#include "maps/MLSGrid.hpp"
#include <Eigen/LU>

#include<kdtree++/kdtree.hpp>

using namespace envire;

struct GridAccess::GridAccessImpl
{
    Environment* env;

    GridAccessImpl(Environment* env) : env(env), grid(NULL) {};

    std::vector<ElevationGrid*> grids;

    ElevationGrid* grid;
    ElevationGrid::ArrayType* gridData;
    FrameNode::TransformType t;
    double z_offset;

    bool evalGridPoint(Eigen::Vector3d& position)
    {
	Eigen::Vector3d local( t * position );
	size_t x, y;
	if( grid->toGrid(local.x(), local.y(), x, y) )
	{
	    position.z() = (*gridData)[y][x] + z_offset;
	    return true;
	}
	return false;
    }

    bool getElevation(Eigen::Vector3d& position)
    {
	// to make this fast, we store the last grid and transform
	// and see try that one first on the next call
	if( grid )
	{
	    if( evalGridPoint( position ) )
		return true;

	    // this is a shortcut, which will only allow one
	    // grid to ever be evaluated. 
	    // TODO: remove this 
	    return false;
	}

	if( grids.size() == 0 )
	    grids = env->getItems<ElevationGrid>();

	for(std::vector<ElevationGrid*>::iterator it = grids.begin();it != grids.end();it++)
	{
	    ElevationGrid* lgrid = *it;
	    FrameNode::TransformType lt =
		env->relativeTransform( 
			env->getRootNode(),
			lgrid->getFrameNode() );

	    grid = lgrid;
	    gridData = &grid->getGridData(ElevationGrid::ELEVATION);
	    t = lt;
	    z_offset = t.inverse(Eigen::Isometry)(2,3);

	    if( evalGridPoint( position ) )
		return true;
	}

	return false;
    }

};

GridAccess::GridAccess(Environment* env)
    : impl( boost::shared_ptr<GridAccessImpl>(new GridAccessImpl(env)) )
{
}

bool GridAccess::getElevation(Eigen::Vector3d& position)
{
    return impl->getElevation( position );
}



struct PointcloudAccess::PointcloudAccessImpl
{
    Environment* env;

    struct TreeNode
    {
	typedef double value_type;

	TreeNode(const Eigen::Vector3d& point) : point(point) {};
	Eigen::Vector3d point;

	inline value_type operator[](size_t n) const
	{
	    return point[n];
	}
    };

    typedef KDTree::KDTree<2, TreeNode> tree_type;

    tree_type kdtree;

    PointcloudAccessImpl(Environment* env) 
	: env(env)
    {
	fillTree(env);
	std::cout << "kdtree inserted points: " << kdtree.size() << std::endl;
    };

    void fillTree(Environment* env)
    {
	std::vector<Pointcloud*> pcs = env->getItems<Pointcloud>();
	for(std::vector<Pointcloud*>::iterator it=pcs.begin();it!=pcs.end();it++)
	{
	    fillTree( *it );
	}
	kdtree.optimize();
    }

    void fillTree(Pointcloud* pc)
    {
	FrameNode::TransformType t =
	    env->relativeTransform( 
		    pc->getFrameNode(),
		    env->getRootNode() );

	for(std::vector<Eigen::Vector3d>::iterator it=pc->vertices.begin();it!=pc->vertices.end();it++)
	{
	    kdtree.insert( TreeNode(t * (*it)) );
	}
    }

    bool getElevation(Eigen::Vector3d& position, double xythresh, double zpos, double zthresh  )
    {
	std::vector<TreeNode> nodes;

	kdtree.find_within_range( TreeNode( position ), xythresh, std::back_insert_iterator<std::vector<TreeNode> >(nodes) );
	for(std::vector<TreeNode>::iterator it=nodes.begin();it!=nodes.end();it++)
	{
	    TreeNode &node(*it);
	    if( abs(node.point.z() - zpos) < zthresh )
	    {
		// for now return the first node found in range
		position = node.point;

		return true;
	    }
	}

	return false;
    }

    bool getElevation(Eigen::Vector3d& position, double threshold )
    {
	std::pair<tree_type::const_iterator,double> found 
	    = kdtree.find_nearest( TreeNode( position ), threshold );

	if( found.first != kdtree.end() )
	{
	    position.z() = (found.first)->point.z();
	    return true;
	}
	else 
	{
	    return false;
	}
    }
};

PointcloudAccess::PointcloudAccess(Environment* env)
    : impl( boost::shared_ptr<PointcloudAccessImpl>(new PointcloudAccessImpl(env)) )
{
}

bool PointcloudAccess::getElevation(Eigen::Vector3d& position, double threshold)
{
    return impl->getElevation( position, threshold );
}

bool PointcloudAccess::getElevation(Eigen::Vector3d& position, double xythresh, double zpos, double zthresh  )
{
    return impl->getElevation( position, xythresh, zpos, zthresh );
}



struct MLSAccess::MLSAccessImpl
{
    Environment* env;

    MLSAccessImpl(Environment* env) : env(env), grid(NULL) {};

    std::vector<MLSGrid*> grids;
    MLSGrid* grid;

    FrameNode::TransformType t;

    bool getElevation(Eigen::Vector3d position, double& zpos, double& zstdev  )
    {
	// to make this fast, we store the last grid and transform
	// and see try that one first on the next call
	if( grid )
	{
	    Eigen::Vector3d v = t * position;
	    zpos = v.z();
	    if( grid->get( v.head<2>(), zpos, zstdev ) )
		return true;

	    // this is a shortcut, which will only allow one
	    // grid to ever be evaluated. 
	    // TODO: remove this 
	    return false;
	}

	if( grids.size() == 0 )
	    grids = env->getItems<MLSGrid>();

	for(std::vector<MLSGrid*>::iterator it = grids.begin();it != grids.end();it++)
	{
	    MLSGrid* lgrid = *it;
	    FrameNode::TransformType lt =
		env->relativeTransform( 
			env->getRootNode(),
			lgrid->getFrameNode() );

	    grid = lgrid;
	    t = lt;

	    Eigen::Vector3d v = t * position;
	    zpos = v.z();
	    if( grid->get( (v).head<2>(), zpos, zstdev ) )
		return true;
	}

	return false;
    }

};

MLSAccess::MLSAccess(Environment* env)
    : impl( boost::shared_ptr<MLSAccessImpl>(new MLSAccessImpl(env)) )
{
}

bool MLSAccess::getElevation(Eigen::Vector3d position, double& zpos, double& zstdev  )
{
    return impl->getElevation( position, zpos, zstdev );
}

