#ifndef __STABILITY_H__
#define __STABILITY_H_

#define EIGEN_USE_NEW_STDVECTOR

#include<Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <math.h>

#include <boost/concept_check.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>


namespace envire {
namespace icp {

class Clustering
{
    public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Clustering( std::vector<Eigen::Transform3d> _points){ 
	    points = _points; mean = Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ); outliners.clear(); translation_covariance.setZero(); rotational_covariance.setZero(); }
	
	/**
	 * The spread is how far can your points be from your mean before it is considered an outliner 
	 * Implemented in 4d, x, y, z, yaw 
	 */ 
	void removeOutliners( Eigen::Vector4d spread);
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
	
	Eigen::Matrix3d getTranslationCovariance() { return translation_covariance; }
	Eigen::Matrix3d getRotationCovariance() { return rotational_covariance; }
	Eigen::Transform3d getMean() { return mean; } 
	std::vector<Eigen::Transform3d> getOutliners() { return outliners; }
	std::vector<Eigen::Transform3d> getPoints() { return points; } 
	unsigned int getSizePoints() { return points.size(); }
	unsigned int getSizeOutliners() { return outliners.size(); }
	
    private: 
	std::vector<Eigen::Transform3d> outliners; 
	std::vector<Eigen::Transform3d> points;
	Eigen::Transform3d mean; 
	Eigen::Matrix3d translation_covariance; 
	Eigen::Matrix3d rotational_covariance; 
	
	

}; 

class Sampling 
{
    public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Sampling() {  sampling_type = -1; } 
	/**
	 * When you ask for a sample it will return a 0 offset 
	 */
	void setModeNoSampling() { sampling_type = 0; } 
	/** 
	* Gets the offset for the 1 sigma search 
	* which means, sampling points at 1 sigma of the estimated position
	* */ 
	void setModeSigmaSampling(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation) 
	    { cov_pos = cov_position; cov_or = cov_orientation; sampling_type = 1;  last_sigma_sample = 0; calcSigmaSamples(); }
	/** 
	* Sets the offset for a uniform generated sample 
	* The search space is min * 1 sigma to max * 1 sigma ( min, max parameters of the uniforma sample generator
	* */ 
	void setModeUniformSampling(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation,  boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > *_generator)
	    { cov_pos = cov_position; cov_or = cov_orientation; sampling_type = 2;  generator = _generator; }
	/**
	 * returns a sample offset based on the sampling mode choosen 
	 */
	Eigen::Transform3d getOffset(); 
	
    private: 
	/**
	 * 0 - No sampling, 0 offset 
	 * 1 - sigma points sampling (9 points) 
	 * 2 - uniform sampling 
	 */
	int sampling_type; 
	Eigen::Matrix3d cov_pos;
	Eigen::Matrix3d cov_or; 
	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > *generator; 
	/**
	 * indicates the last sigma sample returned 
	 */ 
	unsigned int last_sigma_sample; 
	std::vector<Eigen::Transform3d> sigma_samples;
	
	Eigen::Transform3d getUniformSample(); 
	Eigen::Transform3d getSigmaSample(); 
	Eigen::Transform3d getZeroSample(); 
	void calcSigmaSamples(); 
	
}; 


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
	Histogram(double _number_bins, double _area, bool _outliners) 
	    {  number_bins = _number_bins; area = _area; normalization = false; outliners = _outliners; }

	/** 
	 * The pairs_distance data will be normalized ( point - mean / sigma ) 
	 */ 
	void setDataNormalization(double _mean, double _sigma)
	    { mean = _mean; sigma = _sigma; normalization = true; } 
	
	/**
	* gets ths histogram classification based on a linear quernel trained data 
	*/
	double gethistogramSVNClassification( ); 
	std::vector<double> getHistogram() { return histogram; } 
	std::vector<double> getHistogramLimits() { return histogram_limits; } 	
	void calculateHistogram(std::vector<double> pairs_distance);
	
    private:
	std::vector<double> histogram; 
	std::vector<double> histogram_limits; 
	double area; 
	double number_bins;
	int number_of_poins_enviroment; 
	double mean_distance_nearest_neightboar_environment; 
	double mean; 
	double sigma; 
	bool normalization; 
	bool outliners; 
	
	void calculateNormalizedHistogram(std::vector<double> _pairs_distance);
	void calculateNotNormalizedHistogram(std::vector<double> pairs_distance);

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