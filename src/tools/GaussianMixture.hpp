#ifndef __ENVIRE_GAUSSIAN_MIXTURE__
#define __ENVIRE_GAUSSIAN_MIXTURE__

#include <Eigen/Core>

namespace envire
{

template <class _Scalar, int _Dimension>
struct EigenAdapter
{
    enum { Dimension = _Dimension };
    typedef _Scalar Scalar;
    typedef typename Eigen::Matrix<Scalar, Dimension, 1, Eigen::DontAlign> Vector;
    typedef typename Eigen::Matrix<Scalar, Dimension, Dimension, Eigen::DontAlign> Matrix;
};

template <class _Scalar, int _Dimension, class _Adapter = EigenAdapter<_Scalar, _Dimension> >
    struct Gaussian
{
    typedef _Adapter Adapter;
    enum { Dimension = Adapter::Dimension };
    typedef typename Adapter::Scalar Scalar;
    typedef typename Adapter::Vector Vector;
    typedef typename Adapter::Matrix Matrix;

    Vector mean;
    Matrix cov;

    struct Stats
    {
	Matrix Sxx;
	Vector Sx;
	Scalar sum;

	Stats() : Sxx( Matrix::Zero() ), Sx( Vector::Zero() ), sum(0.0) {}
	void insert( const Vector& v, Scalar weight = 1.0 )
	{
	    Sx += v * weight;
	    Sxx += v * v.transpose() * weight; 
	    sum += weight;
	}

	Gaussian<_Scalar, _Dimension, Adapter> update()
	{
	    Vector mean = Sx / sum;
	    Matrix cov = Sxx / sum - mean*mean.transpose(); 
	    return Gaussian<_Scalar, _Dimension, Adapter>( mean, cov );
	}
    };

    Gaussian() {}
    Gaussian( const Vector& mean, const Matrix& cov )
	: mean( mean ), cov( cov ) {}

    Scalar eval( const Vector& x ) const
    {
	Vector d = x - mean;
	Eigen::Matrix<Scalar,1,1> e = d.transpose() * cov.inverse() * d;
	Scalar res = 
	    pow( 2.0 * M_PI, -Dimension/2.0 )
	    * sqrt( cov.determinant() )
	    * exp( -0.5 * e[0] );

	return res;
    }
};

template <class _Scalar, int _Dimension, class _Adapter = EigenAdapter<_Scalar, _Dimension> >
    class GaussianMixture
{
    
public:
    typedef _Adapter Adapter;
    enum { Dimension = Adapter::Dimension };
    typedef typename Adapter::Scalar Scalar;
    typedef typename Adapter::Vector Vector;
    typedef typename Adapter::Matrix Matrix;

    struct Parameter 
    {
	enum { Dimension = Adapter::Dimension };
	typedef typename Adapter::Scalar Scalar;
	typedef typename Adapter::Vector Vector;
	typedef typename Adapter::Matrix Matrix;

	Parameter() {}
	Parameter( Scalar weight, const Vector& mean, const Matrix& cov )
	    : dist( mean, cov ), weight( weight ) {}
	Parameter( Scalar weight, const Gaussian<Scalar, Dimension, Adapter>& g )
	    : dist( g ), weight( weight ) {}

	Gaussian< _Scalar, _Dimension, Adapter> dist;
	Scalar weight;
    };
    typedef std::vector<Parameter> Parameters;
    Parameters params;

    /** normalize the weights of the gaussians, so that the sum is 1.0 
     */
    void normalize()
    {
	Scalar sum = 0.0;
	for( size_t n=0; n<params.size(); n++ )
	    sum += params[n].weight; 

	for( size_t n=0; n<params.size(); n++ )
	{
	    if( sum > 1e-5 )
		params[n].weight /= sum; 
	    else
		params[n].weight = 1.0 / params.size();
	}
    }

    Scalar eval( const Vector& s ) const
    {
	Scalar v = 0;
	for( size_t j=0; j<params.size(); j++ )
	    v += params[j].dist.eval( s );

	return v;
    }
};

}

#endif
