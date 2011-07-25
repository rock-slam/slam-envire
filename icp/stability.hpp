#ifndef __STABILITY_H__
#define __STABILITY_H_

#define EIGEN_USE_NEW_STDVECTOR

#include<Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <math.h>

#include <boost/concept_check.hpp>

#include "boost/shared_ptr.hpp"
#include "boost/random.hpp"
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <ctime>  

#include "icpConfigurationTypes.hpp"

namespace envire {
namespace icp {

class SigmaPoints
{
    public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	* Calculates the n times sigma points of a matrix, 
	* where the sigma points norm should be within a bounded limit.  
	*/
	Eigen::Matrix3d calcSigmaPoints(Eigen::Matrix3d matrix, SigmaPointConfiguration conf)
	{
	    Eigen::LLT<Eigen::Matrix3d> llt;
	    llt.compute(matrix);
	    Eigen::Matrix3d sigma_points = llt.matrixL();

	    sigma_points = conf.n_sigma * sigma_points; 

	    for(int column = 0; column < 3; column++) 
	    {	
		if( sigma_points.col(column).norm() < conf.min_norm ) 
			sigma_points.col(column) = (Eigen::Matrix3d::Identity() * conf.min_norm).col(column) ;
		if( sigma_points.col(column).norm() > conf.max_norm ) 
			sigma_points.col(column) = (Eigen::Matrix3d::Identity() * conf.max_norm).col(column) ; 
	    }
// 	    std::cout << " Matrix " << " min " << conf.min_norm << " max " <<conf.max_norm<< " n " << conf.n_sigma<<std::endl; 
// 	    std::cout << matrix << std::endl; 
// 	    std::cout << " LLT " << std::endl; 
// 	    std::cout << llt.matrixL() << std::endl; 
// 	    std::cout << " sigma" << std::endl; 
// 	    std::cout << sigma_points << std::endl; 
	    return sigma_points;
	}
};


/** 
 * Cluster transforms 3d in x,y,z and yaw !!! 
 */ 
class Clustering
{
    public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Clustering(ClusteringConfiguration conf){ 
	     this->conf=conf; translation_covariance.setZero(); rotational_covariance.setZero(); }
	
	/** 
	 * Calculates the spread for the outliner removal 
	 */ 
	void defOutlinerRegion(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation); 
	
	bool cluster(  std::vector<Eigen::Affine3d> points ); 
	
	Eigen::Matrix3d getTranslationCovariance() { return translation_covariance; }
	Eigen::Matrix3d getRotationCovariance() { return rotational_covariance; }
	Eigen::Affine3d getMean() { return mean; } 
	std::vector<Eigen::Affine3d> getPoints() { return points; } 
	unsigned int getSizePoints() { return points.size(); }
	
    private: 
	ClusteringConfiguration conf; 
	std::vector<Eigen::Affine3d> points;
	Eigen::Affine3d mean; 
	Eigen::Matrix3d translation_covariance; 
	Eigen::Matrix3d rotational_covariance; 
	Eigen::Matrix3d variance_limit_pos;  
	Eigen::Matrix3d variance_limit_ori;
	
	/** 
	* Calculates variance of a number of samples 
	* Variance in Translation 
	* Need to calculate MEAN before calculating Variance 
	*/ 
	void calcVariance( ); 
	/** 
	* Calculates the mean of a number of points 
	*/ 
	void calcMean(); 
	/**
	 * The spread is how far can your points be from your mean before it is considered an outliner 
	 * Implemented in 4d, x, y, z, yaw 
	 * The spread is considered a % of the 1 sigma distribution 
	 * It is considered within a Maximal and Minimal size defined 
	 */ 
	bool removeOutliers( );

	
	
	

}; 


class Sampling 
{
    public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Sampling( SamplingConfiguration conf ) 
	{  
	    this->conf = conf;  

	} 
	

	void defSearchRegion(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation);  

	/**
	 * returns a sample offset based on the sampling mode choosen 
	 */
	Eigen::Affine3d getUniformOffset(); 
	Eigen::Affine3d getSigmaOffset(); 
	
	Eigen::Matrix3d getSigmaPointsOfSamplingRegionTranslation(){ return sigmaPointsPosition;}
	Eigen::Matrix3d getSigmaPointsOfSamplingRegionRotation(){return sigmaPointsOrientation;} 
	
    private: 
	
	double getRandomValue(int min, int max)
	{
	    static boost::minstd_rand gen((unsigned int)std::time(NULL));
	    boost::uniform_real<>  dist(min, max);
	    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > generator(gen, dist);
	    return generator();
	}
	SamplingConfiguration conf; 
	static boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > generator; 
	/**
	 * indicates the last sigma sample returned 
	 */ 
	unsigned int last_sigma_sample; 
	std::vector<Eigen::Affine3d> sigma_samples;
	
	Eigen::Affine3d getUniformSample(); 
	Eigen::Affine3d getSigmaSample(); 
	Eigen::Affine3d getZeroSample(); 
	void calcSigmaSamples(); 
	void calcSigmaPoints(Eigen::Matrix3d cov_pos, Eigen::Matrix3d cov_or, double min_distance, double min_angle);
	Eigen::Matrix3d sigmaPointsPosition; 
	Eigen::Matrix3d sigmaPointsOrientation; 
	
}; 

//TODO FIX THE CONFIGURATION 
class Histogram 
{ 
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/**
	 * Creates a histogram distribution based on a number of points 
	 * _number_bins is the number of bins in the distribution 
	 * _area is the size of the bin 
	 * , //TODO RIGHT NOW AREA IS THE TOTAL AREA FIX 
	 * _if the outliners is true, 2 extra bins will be created to the number of bins where the outliners will be stored
	 * the histogram is normalized so if the oultiners are considered this will impact the distribution
	 */
	Histogram(HistogramConfiguration conf) 
	    {  this->conf = conf; calculateBinLimits(conf.n_sigma, conf.number_bins); }

	/**
	* gets ths histogram classification based on a linear quernel trained data 
	*/
	double getSVMValue( ){ return svm_value; } 
	std::vector<double> getHistogram() { return histogram; } 
	std::vector<double> getHistogramLimits() { return histogram_limits; } 	
	
	void calculateHistogram( std::vector<double> pairs_distance ) ;
	
	
    private:
	double svm_value; 
	HistogramConfiguration conf; 
	std::vector<double> histogram; 
	std::vector<double> histogram_limits; 
	void calculateBinLimits( int n_sigma, int number_bins ); 
	void calculateStandartGaussianHistogram(std::vector<double> _pairs_distance); 
	double calcSVMValue( ); 
// 		//for normalization 
// 	//In theory the mean distance to nearest neighbor in an infinitly large random distribution is 
// 	// re = 1 / (2 * sqrt ( density ) and the sandart deviation  0.26136 / sqrt ( N * density ) 
// 	// where N number of points 
// 	//so density can be given by 
// 	double mean = 0.1; 
// 	double density = pow( 1/(2*0.1), 2 ); 
// 	//so the standart deviation is 
// 	//double sigma = 0.26136 / sqrt( density);
// 	double sigma = 0.26136; 
// 	Histogram histogram( 8, 4*sigma); 
}; 

}
}

#endif
