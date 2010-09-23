#ifndef __ICP_H__
#define __ICP_H__

#define EIGEN_USE_NEW_STDVECTOR

#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/StdVector>
#include<Eigen/QR>
#include<Eigen/LU>

#include<kdtree++/kdtree.hpp>

#include<envire/Core.hpp>
#include<envire/Pointcloud.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <utility>

namespace envire {
namespace icp {

class Pairs
{
public:
    const static unsigned int MIN_PAIRS = 3;

    /** add a single pair, and the distance between that a and b
     */
    void add( const Eigen::Vector3d& a, const Eigen::Vector3d& b, double dist );

    /** trim the pairs to the @param n_po pairs with the lowest distance.
     * Will @return the largest distance of those @param n_po pairs.
     */
    double trim( size_t n_po );

    /** will return the transform that has to be applied to B, so that the MSE
     * between the invididual pairs of A and B is minimized.
     */
    Eigen::Transform3d getTransform();

    double getMeanSquareError() const;

    /** will return the number of pairs in the object
     */
    size_t size() const;

    /** remove all pairs */
    void clear();

private:
    std::vector<Eigen::Vector3d> x, p;

    struct pair
    {
	size_t index;
	double distance;

	bool operator < ( const pair other ) const
	{
	    return distance < other.distance;
	}
    };
    std::vector<pair> pairs;

    double mse;
};

struct VertexNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef double value_type;

    Eigen::Matrix<value_type,3,1> point;

    inline value_type operator[](size_t n) const
    {
	return point[n];
    }
};

struct VertexEdgeAndNormalNode : public VertexNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<value_type,3,1> normal;
    bool edge;
};

class PointcloudAdapter
{
public:
    PointcloudAdapter( envire::Pointcloud* model, double density )
	: model(model), index( 0.0 ),
	vertices( &model->vertices ), 
	density( density )
    {
	envire::FrameNode* fm = model->getFrameNode();
	envire::Environment* env = model->getEnvironment();

	assert( fm );
	assert( env );

	// get the transformation to the root framenode
	C_local2global = env->relativeTransform(
		fm,
		env->getRootNode() );

	C_local2globalnew = C_local2global;
    }

    void setOffsetTransform( const Eigen::Transform3d& C_globalnew2global )
    {
	C_local2globalnew = C_globalnew2global.inverse() * C_local2global;
    }

    VertexNode next()
    {
	VertexNode n;
	const size_t idx = index;
	n.point = C_local2globalnew * (*vertices)[idx];
	index += 1.0/density;
	return n;
    }
    bool hasNext() const
    {
	const size_t idx = index;
	return idx < vertices->size();
    }
    void reset() 
    {
	index = 0.0;
    }

    void applyTransform(const Eigen::Transform3d& t)
    {
	envire::FrameNode* fn = model->getFrameNode();
	fn->setTransform( envire::FrameNode::TransformType( fn->getTransform()*(C_local2global.inverse() * t * C_local2global ) ) );
    }

protected:
    envire::Pointcloud* model;
    Eigen::Transform3d C_local2global, C_local2globalnew;

    double index;
    const std::vector<Eigen::Vector3d> *vertices;
    double density;
};

class PointcloudEdgeAndNormalAdapter : public PointcloudAdapter
{
public:
    PointcloudEdgeAndNormalAdapter( envire::Pointcloud* model, double density )
	: PointcloudAdapter( model, density ) 
    {
	attrs = &model->getVertexData<envire::Pointcloud::vertex_attr>(envire::Pointcloud::VERTEX_ATTRIBUTES);
	normals = &model->getVertexData<Eigen::Vector3d>(envire::Pointcloud::VERTEX_NORMAL);

    }

    VertexEdgeAndNormalNode next() 
    {
	VertexEdgeAndNormalNode n;
	const size_t idx = index;
	n.point = C_local2globalnew * (*vertices)[idx];
	n.edge = (*attrs)[idx] & (1 << envire::Pointcloud::SCAN_EDGE);
	n.normal = C_local2globalnew.rotation() * (*normals)[idx];
	index += 1.0/density;
	return n;
    }

private:
    std::vector<Eigen::Vector3d> *normals;
    std::vector<envire::Pointcloud::vertex_attr> *attrs;
};

template <class T>
struct PairFilter
{
    inline bool operator()(const T& a, const T& b) const { return true; }
};

struct EdgeAndNormalPairFilter
{
    static const double MAX_NORMAL_DEV = M_PI/8.0;
    inline bool operator()(const VertexEdgeAndNormalNode& a, const VertexEdgeAndNormalNode& b) const
    {
	const bool edge = (a.edge || b.edge);
	const double normal_angle = 
	    acos( std::min( 1.0, a.normal.dot( b.normal )) );

	return !edge && (normal_angle < MAX_NORMAL_DEV );
    }
};

template <class _TreeNode, class _Adapter, class _Filter = PairFilter<_TreeNode> >
class FindPairsKDTree
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void addModel( _Adapter& model )
    {
	model.reset();
	while( model.hasNext() )
	    kdtree.insert( model.next() );
    }
    
    void findPairs( _Adapter& model, Pairs& pairs, double d_box )
    {
	model.reset();
	while( model.hasNext() )
	{
	    _TreeNode node = model.next();
	    std::pair<typename tree_type::const_iterator,double> found = kdtree.find_nearest(node, d_box);
	    if( found.first != kdtree.end() && filter(node, *(found.first)) )
		pairs.add(node.point, (found.first)->point, found.second);
	}
    }

    void clear()
    {
	kdtree.clear();
    }

private:
    typedef KDTree::KDTree<3, _TreeNode> tree_type;

    _Filter filter;
    tree_type kdtree;
};


template <class _Adapter, class _FindPairs>
class Trimmed {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** performs a single alignment of the measurement to the model.
     * The model needs to be added using addToModel before this call.
     * 
     * @param measurement - the mesh that needs to be matched
     * @param max_iter - maximum number of iterations
     * @param min_mse - minimum average square distance between points after which to stop
     * @param min_mse_diff - minimum difference in average square distance between points after which to stop
     */
    void align( _Adapter measurement, size_t max_iter, double min_mse, double min_mse_diff )
    {
	Pairs pairs;
	Eigen::Transform3d C_globalc2global( Eigen::Transform3d::Identity() );

	iter = 0;
	double d_box = std::numeric_limits<double>::infinity();
	mse_diff = mse = std::numeric_limits<double>::infinity();
	double old_mse = mse;
	while( iter < max_iter && mse > min_mse && mse_diff > min_mse_diff )
	{
	    old_mse = mse;

	    pairs.clear();
	    findPairs.findPairs( measurement, pairs, d_box );
	    // d_box = pairs.trim( sum );
	    Eigen::Transform3d C_globalc2globalp = pairs.getTransform();

	    C_globalc2global = C_globalc2global * C_globalc2globalp;
	    measurement.setOffsetTransform( C_globalc2global );

	    mse = pairs.getMeanSquareError();
	    mse_diff = old_mse - mse;

	    iter++;

	    std::cout << "iter: " << iter
		    << "\tpairs: " << pairs.size()
		    << "\tmse: " << mse
		    << "\tmse_diff: " << mse_diff
		    << "\td_box: " << d_box
		    << std::endl;
	}
    }

    /** adds the @param model trimesh to the ICP model
     * 
     * @param model - model to be added
     */
    void addToModel( _Adapter model )
    {
	findPairs.addModel( model );
    }
    
    /** resets the model 
     */
    void clearModel() { findPairs.clear(); }

    size_t getNumIterations() { return iter; }
    double getMeanSquareError() { return mse; }
    double getMeanSquareErrorDiff() { return mse_diff; }
    double getOverlap() { return overlap; }

private:
    size_t iter;
    double mse;
    double mse_diff;
    double overlap;

    _FindPairs findPairs;
};

typedef FindPairsKDTree< VertexEdgeAndNormalNode,
	PointcloudEdgeAndNormalAdapter, EdgeAndNormalPairFilter >
	FindPairsKDEAN;

typedef FindPairsKDTree< VertexNode,
	PointcloudAdapter >
	FindPairsKD;

typedef Trimmed< PointcloudEdgeAndNormalAdapter, FindPairsKDEAN > TrimmedKDEAN;
typedef Trimmed< PointcloudAdapter, FindPairsKD > TrimmedKD;

}
}

#endif
