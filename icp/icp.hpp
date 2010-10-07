#ifndef __ICP_H__
#define __ICP_H__

#define EIGEN_USE_NEW_STDVECTOR

#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/StdVector>
#include<Eigen/QR>
#include<Eigen/LU>

#include<kdtree++/kdtree.hpp>

#include "../envire/Core.hpp"
#include "../envire/Pointcloud.hpp"

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/random/variate_generator.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

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

    void setOffsetTransform( const Eigen::Transform3d& C_global2globalnew )
    {
	C_local2globalnew = C_global2globalnew * C_local2global;
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
    size_t size() const
    {
	return vertices->size() * density;
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
		pairs.add((found.first)->point, node.point, found.second);
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

template <class T>
struct GoldenBracket
{
    template <class B>
    static inline void SHIFT2( B& a, B& b, B c )
    {
	a = b;
	b = c;
    }

    template <class B>
    static inline void SHIFT3( B& a, B& b, B& c, B d )
    {
	a = b;
	b = c;
	c = d;
    }

    static T findMin( boost::function<T (double)> f, double ax, double bx, double cx, double eps )
    {
	// implementation from numerical recipies
	const double R = 0.6180339;
	const double C = (1.0-R);
	T f1,f2;
	double x0,x1,x2,x3;

	x0 = ax;
	x3 = cx;
	if( fabs(cx-bx) > fabs(bx-ax) )
	{
	    x1 = bx;
	    x2 = bx + C*(cx-bx);
	}
	else
	{
	    x2 = bx;
	    x1 = bx - C*(bx-ax);
	}
	f1 = f(x1);
	f2 = f(x2);
	while (fabs(x3-x0) > eps*(fabs(x1)+fabs(x2))) 
	{
	    if (f2 < f1) {
		SHIFT3(x0,x1,x2,R*x1+C*x3);
		SHIFT2(f1,f2,f(x2));
	    } 
	    else 
	    {
		SHIFT3(x3,x2,x1,R*x2+C*x0);
		SHIFT2(f2,f1,f(x1));
	    }
	}
	if (f1 < f2) 
	{
	    return f1;
	} 
	else 
	{
	    return f2;
	}
    }
};

template <class _Adapter, class _FindPairs>
class Trimmed {
    struct Result
    {
	const static double gamma = 2.0;

	size_t iter;
	size_t pairs;
	double mse;
	double mse_diff;
	double d_box;
	double overlap;
	Eigen::Transform3d C_global2globalnew;

	double optFunc() const { return mse/pow(overlap, 1.0+gamma); }
	bool operator< (const Result& other) const { return optFunc() < other.optFunc(); }
	double operator+ (const Result& other) const { return optFunc() + other.optFunc(); }
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** performs an iterative alignment of the measurement to the model.
     * The model needs to be added using addToModel before this call.
     * This method performs a golden section search for the optimal overlap
     * parameters based. The implementation is based on the TrimmedICP
     * publication by Chetverikov.
     * 
     * @param measurement - the pointcloud that needs to be matched
     * @param max_iter - maximum number of iterations
     * @param min_mse - minimum average square distance between points after which to stop
     * @param min_mse_diff - minimum difference in average square distance between points after which to stop
     * @param alpha - start of interval to search for overlap
     * @param beta - end of interval to search for overlap
     * @param eps - accuracy for for searching for the overlap parameters
     */
    void align( _Adapter measurement, size_t max_iter, double min_mse, double min_mse_diff, double alpha, double beta, double eps )
    {
	typedef Trimmed<_Adapter, _FindPairs> t;

	boost::function<Result (double)> evalfunc =
		boost::bind( &t::_align, this, measurement, max_iter, min_mse, min_mse_diff, _1 );

	minResult = GoldenBracket<Result>::findMin( 
		evalfunc,
		alpha,
		(alpha + beta)/2.0,
		beta,
		eps);

	measurement.applyTransform( minResult.C_global2globalnew );
    }

    /** performs an iterative alignment of the measurement to the model.
     * The model needs to be added using addToModel before this call.
     * 
     * @param measurement - the pointcloud that needs to be matched
     * @param max_iter - maximum number of iterations
     * @param min_mse - minimum average square distance between points after which to stop
     * @param min_mse_diff - minimum difference in average square distance between points after which to stop
     * @param overlap - percentage of overlap, range between [0..1]. A value of
     *                  0.95 will discard 5% of the pairs with the worst matches
     */
    void align( _Adapter measurement, size_t max_iter, double min_mse, double min_mse_diff, double overlap )
    {
	minResult = _align( measurement, max_iter, min_mse, min_mse_diff, overlap );
	measurement.applyTransform( minResult.C_global2globalnew );
    }

    /** adds the @param model pointcloud to the ICP model
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

    size_t getNumIterations() { return minResult.iter; }
    double getMeanSquareError() { return minResult.mse; }
    double getMeanSquareErrorDiff() { return minResult.mse_diff; }
    double getOverlap() { return minResult.overlap; }
    size_t getPairs() { return minResult.pairs; }

private:
    /** performs a single alignment of the measurement to the model.
     * The model needs to be added using addToModel before this call.
     * 
     * @param measurement - the mesh that needs to be matched
     * @param max_iter - maximum number of iterations
     * @param min_mse - minimum average square distance between points after which to stop
     * @param min_mse_diff - minimum difference in average square distance between points after which to stop
     * @param overlap - percentage of overlap, range between [0..1]. A value of
     *                  0.95 will discard 5% of the pairs with the worst matches
     */
    Result _align( _Adapter measurement, size_t max_iter, double min_mse, double min_mse_diff, double overlap )
    {
	Result result;
	Pairs pairs;
	result.C_global2globalnew = Eigen::Transform3d::Identity();

	result.iter = 0;
	result.overlap = overlap;
	result.d_box = std::numeric_limits<double>::infinity();
	result.mse_diff = result.mse = std::numeric_limits<double>::infinity();
	double old_mse = result.mse;
	while( result.iter < max_iter && result.mse > min_mse && result.mse_diff > min_mse_diff )
	{
	    old_mse = result.mse;

	    pairs.clear();
	    findPairs.findPairs( measurement, pairs, result.d_box );
	    const size_t n_po = measurement.size() * result.overlap;
	    result.d_box = pairs.trim( n_po ) * 2.0;
	    result.pairs = pairs.size();
	    if( result.pairs < Pairs::MIN_PAIRS )
		return result;

	    Eigen::Transform3d C_globalprev2globalnew = pairs.getTransform();
	    result.C_global2globalnew = C_globalprev2globalnew * result.C_global2globalnew;
	    measurement.setOffsetTransform( result.C_global2globalnew );

	    result.mse = pairs.getMeanSquareError();
	    result.mse_diff = old_mse - result.mse;

	    result.iter++;

	    /*
	std::cout
	    << "points: " << measurement.size()
	    << "\titer: " << result.iter
	    << "\tpairs: " << pairs.size()
	    << "\tmse: " << result.mse
	    << "\tmse_diff: " << result.mse_diff
	    << "\td_box: " << result.d_box
	    << "\toverlap: " << result.overlap
	    << std::endl;
	    */
	}
	/*
	std::cout
	    << std::endl;
	    */


	return result;
    }

    _FindPairs findPairs;
    Result minResult;
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
