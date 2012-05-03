#ifndef __ENVIRE_EXPECTATION_MAXIMIZATION__
#define __ENVIRE_EXPECTATION_MAXIMIZATION__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <numeric>
#include <kdtree++/kdtree.hpp>

#include <envire/tools/GaussianMixture.hpp>

namespace envire
{

template <class GMM>
    class KMeans
{
public:
    enum { Dimension = GMM::Dimension };
    typedef typename GMM::Scalar Scalar;
    typedef typename GMM::Adapter::Vector Vector;
    typedef typename GMM::Adapter::Matrix Matrix;

protected:
    struct Sample : public Vector
    {
	typedef Scalar value_type;

	Sample() {};
	Sample( size_t idx, const Vector& v )
	    : Vector( v ), idx( idx ) {}
	size_t idx;
    };

    typedef typename KDTree::KDTree<Dimension, Sample> tree_type;
    tree_type kdtree;

    std::vector<Vector> samples;
    std::vector<Scalar> weights;
    std::vector<int> index;

    std::vector<Scalar> sum;
    std::vector<Vector> means;

public:
    void initialize( size_t numClasses, const std::vector<Vector>& s, const std::vector<Scalar>& w = std::vector<Scalar>() )
    {
	assert( numClasses <= s.size() && numClasses > 0 );

	// copy samples 
	samples.resize( s.size() );
	std::copy( s.begin(), s.end(), samples.begin() );

	weights.resize( w.size() );
	std::copy( w.begin(), w.end(), weights.begin() );

	// create index which stores class assignment for each 
	// sample. assign evenly in the beginning
	index.resize( s.size(), 0 );

	// initialize the means kdtree and array
	kdtree.clear();
	const int stride = s.size() / numClasses;
	for( size_t j=0; j<numClasses; j++ )
	    kdtree.insert( Sample( j, s[j*stride]) );

	// array is only used when calculating the new means
	means.resize( numClasses );
	sum.resize( numClasses );
    }

    void calcMeans()
    {
	// calc the means for each class
	const size_t numClasses = means.size();
	std::fill( means.begin(), means.end(), Vector::Zero() );
	std::fill( sum.begin(), sum.end(), 0.0 );
	bool hasWeights = weights.size();
	for( size_t i=0; i<samples.size(); i++ )
	{
	    const size_t idx = index[i];
	    const Scalar weight = hasWeights ? weights[i] : 1.0;

	    means[idx] += samples[i] * weight;
	    sum[idx] += weight;
	}

	// store the means in the kdtree
	kdtree.clear();
	for( size_t j=0; j<numClasses; j++ )
	{
	    means[j] /= sum[j];
	    kdtree.insert( Sample( j, means[j]) );
	}
    }

    size_t associateSamples()
    {
	size_t switches = 0;
	for( size_t i=0; i<samples.size(); i++ )
	{
	    std::pair<typename tree_type::const_iterator, Scalar> res = kdtree.find_nearest( Sample(0, samples[i]) );
	    const int classIdx = res.first->idx;
	    switches += (classIdx != index[i]);
	    index[i] = classIdx;
	}
	return switches;
    }

    void run()
    {
	while( associateSamples() > 0 )
	    calcMeans();
    }

    std::vector<Gaussian<Scalar, Dimension, typename GMM::Adapter> > getClusters() 
    {
	// sum up the individual samples
	bool hasWeights = weights.size();
	std::vector<typename Gaussian<Scalar, Dimension, typename GMM::Adapter>::Stats > stats( means.size() );
	for( size_t i=0; i<samples.size(); i++ )
	    stats[index[i]].insert( samples[i], hasWeights ? weights[i] : 1.0 );

	// and calc mean and covariance
	std::vector<Gaussian<Scalar, Dimension, typename GMM::Adapter> > res( means.size() );
	for( size_t j=0; j<means.size(); j++ )
	    res[j] = stats[j].update();
	return res;
    }
};

template <class GMM>
class GaussianMixtureSampling
{
public:
    enum { Dimension = GMM::Dimension };
    typedef typename GMM::Scalar Scalar; 
    typedef typename GMM::Matrix Matrix;
    typedef typename GMM::Vector Vector;
    GMM& gmm;

    boost::minstd_rand rand_gen;
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand_uni;

    GaussianMixtureSampling( GMM& gmm, long seed = 42u )
	: gmm( gmm ), rand_gen( seed ),
	  rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
	  rand_uni(rand_gen, boost::uniform_real<>(0,1.0) )
    {
    }

    Vector sample()
    {
	Vector sample;
	for( int i=0; i<Dimension; i++ )
	    sample[i] = rand_norm();

	Scalar r = rand_uni();
	Scalar s = 0.0;
	for( size_t j=0; j<gmm.params.size(); j++ )
	{
	    typename GMM::Parameter &param( gmm.params[j] );
	    s += param.weight;
	    if( s >= r )
		return param.dist.mean + param.dist.cov.llt().matrixL() * sample;
	}

	throw std::runtime_error("distribution not normalized.");
    }
};

template <class GMM>
class ExpectationMaximization
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum { Dimension = GMM::Dimension };
    typedef typename GMM::Scalar Scalar;
    typedef typename GMM::Matrix Matrix;
    typedef typename GMM::Vector Vector;

    GMM gmm;
    std::vector<Vector> samples;
    std::vector<Scalar> weights;

    void run( Scalar delta, size_t max_iter )
    {
	size_t iter = 0;
	Scalar ll_diff = 1e5;
	Scalar last_ll = 0; 

	while( iter < max_iter && ll_diff > delta )
	{
	    Scalar ll = step();
	    ll_diff = abs( ll - last_ll );
	    last_ll = ll;
	    iter++;
	}
    }

    Scalar step()
    {
	//
	// algorithm is based on the document
	// EM Demystified: An Expectation-Maximization Tutorial
	// https://www.ee.washington.edu/techsite/papers/documents/UWEETR-2010-0002.pdf
	//
	// and extended for weighted samples
	//

	const size_t k = gmm.params.size();
	const size_t n = samples.size();
	if( k == 0 || n < k )
	    return 0;

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> gamma((int)n, (int)k);

	// true if the samples are weighted
	bool weighted = weights.size();

	// evaluate the current gmm
	Scalar loglhood = 0;
	for( size_t i = 0; i < n; i++ )
	{
	    Scalar sum = 0;
	    for( size_t j = 0; j < k; j++ )
	    {
		typename GMM::Parameter &param( gmm.params[j] );
		Scalar val = param.weight * param.dist.eval( samples[i] );
		gamma(i,j) = val;
		sum += val;
	    }
	    loglhood += log( sum );
	    gamma.row(i) *= (weighted ? weights[i] : 1.0) / sum;
	}
	loglhood /= n;

	const Scalar remove_threshold = 1e-4;
	std::vector<size_t> remove;
	// update the parameters of the gaussians
	Scalar sum_weights = weighted ? std::accumulate( weights.begin(), weights.end(), 0.0 ) : n;
	for( size_t j = 0; j < k; j++ )
	{
	    Scalar n_j = gamma.col(j).sum();
	    typename GMM::Parameter &param( gmm.params[j] );
	    param.weight = n_j / sum_weights;
	    if( n_j < remove_threshold || n_j != n_j )
		remove.push_back( j );

	    Vector mean = Vector::Zero();
	    for( size_t i = 0; i < n; i++ )
		mean += gamma(i,j) * samples[i];
	    param.dist.mean = 1.0/n_j * mean;

	    Matrix cov = Matrix::Zero();
	    for( size_t i = 0; i < n; i++ )
	    {
		Vector c = samples[i] - param.dist.mean;
		cov += gamma(i,j) * c * c.transpose();
	    }
	    param.dist.cov = 1.0/n_j * cov;
	}

	std::sort( remove.begin(), remove.end() );
	for( std::vector<size_t>::reverse_iterator it = remove.rbegin(); it != remove.rend(); it++ )
	{
	    gmm.params.erase( gmm.params.begin() + *it );
	}

	return loglhood;
    }

    void initialize( size_t numClasses, const std::vector<Vector>& s, const std::vector<Scalar>& w = std::vector<Scalar>() )
    {
	// copy samples and weights
	samples.resize( s.size() );
	std::copy( s.begin(), s.end(), samples.begin() );

	weights.resize( w.size() );
	std::copy( w.begin(), w.end(), weights.begin() );

	// if weights are given, they need to be the same size as the samples
	assert( !w.size() || w.size() == s.size() );

	// perform a k-means clustering for finding the initial values
	KMeans<GMM> km;
	km.initialize( numClasses, s, w );
	km.run();

	std::vector<Gaussian<Scalar, Dimension, typename GMM::Adapter> > 
	    cluster = km.getClusters();
	
	gmm.params.clear();
	for( size_t i=0; i<cluster.size(); i++ )
	{
	    typename GMM::Parameter p( 1.0, cluster[i] );
	    gmm.params.push_back( p ); 
	}

	// normalize the weights 
	gmm.normalize();
    }
};

}

#endif
