#include "stability.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/SVD>
//#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <iostream>

using namespace std; 
using namespace Eigen; 
using namespace envire::icp;


bool Clustering::cluster(  std::vector<Eigen::Affine3d> points ) 
{
    this->points = points;
    
    if( points.size() < conf.min_number_of_points )
	return false;
    
    if( conf.remove_outliers ) 
    {
	return removeOutliers( );
    }
    else
    {
	calcMean();
	calcVariance();
	return true; 
    }
    
    
}
void Clustering::calcVariance(  )
{

    //find the variance, variables considered independant 
    Vector3d variance_postion; 
    variance_postion.setZero(); 
    double variance_yaw = 0; 
    
    for(unsigned int i = 0; i < points.size(); i++) 
    {
	variance_postion = variance_postion + (mean.translation() - points.at(i).translation()).array().square().matrix(); 
	variance_yaw = variance_yaw + pow(  Matrix3d( mean.rotation() ).eulerAngles(2,1,0)[0]  -  Matrix3d( points.at(0).rotation()).eulerAngles(2,1,0)[0], 2 );
    }

    variance_postion = variance_postion / points.size();
    variance_yaw = variance_yaw / points.size(); 
    
    //cout << " variance " << endl; 
    //cout << variance_postion.transpose() << " - " << variance_yaw * 180 / M_PI << endl; 
    
    translation_covariance = Eigen::Matrix3d::Zero() ;

    //sets a minimum variance 
    for( int i = 0; i < 3; i++ ) 
    {
	translation_covariance(i,i) = variance_postion(i); 
    }
    
    rotational_covariance = Matrix3d::Zero(); 
    rotational_covariance(2,2) = variance_yaw; 

}


void Clustering::calcMean( )
{
    //find the mean of position and rotation solution 
    Vector3d position_mean;
    double yaw_mean;
    position_mean.setZero();
    yaw_mean = 0; 
    
    for(unsigned int i = 0; i < points.size(); i++) 
    {
	//std::cout << points.at(i).translation().transpose() << " - " <<  Matrix3d( points.at(i).rotation()).eulerAngles(2,1,0)[0] * 180 / M_PI<<  endl; 
	position_mean = position_mean + points.at(i).translation(); 
	yaw_mean = yaw_mean + Matrix3d( points.at(i).rotation()).eulerAngles(2,1,0)[0]; 
	
    }
    
    position_mean = position_mean /  points.size(); 
    yaw_mean = yaw_mean /  points.size(); 
    //std::cout <<points.size() <<" mean " << position_mean.transpose() << " - " <<  yaw_mean * 180 / M_PI <<  endl; 
    mean = Eigen::AngleAxisd( yaw_mean, Eigen::Vector3d::UnitZ() ); 
    mean.translation() = position_mean;

}

void Clustering::defOutlinerRegion(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation)
{
    if ( conf.remove_outliers ) 
    {
	SigmaPoints sigma_points; 
	Eigen::Matrix3d sigma_points_pos = sigma_points.calcSigmaPoints(cov_position, conf.outliers_position); 
	Eigen::Matrix3d sigma_points_ori = sigma_points.calcSigmaPoints(cov_position, conf.outliers_orientation); 
	
	variance_limit_ori = sigma_points_ori * sigma_points_ori; 
	variance_limit_pos = sigma_points_pos * sigma_points_pos; 
    }
}

bool Clustering::removeOutliers(  )
{
  cout << " Variance Limit " << variance_limit_pos.norm()<< endl; 
  cout << variance_limit_pos << endl; 
  cout << variance_limit_ori(2,2) << endl; 
    //cout << "spread " << spread.segment<3>(0).transpose() << " " <<spread(3) * 180 / M_PI<< endl; 
    while( points.size() >= conf.min_number_of_points )
    {
	calcMean( ); 
	calcVariance( );
	cout << " Number of Points " << points.size() << endl; 
	cout << translation_covariance << std::endl; 
	cout << translation_covariance.norm() << " - " << rotational_covariance(2,2) * 180 / M_PI << endl;
	//verify if the variance is within the cluestering limits 
	int at_max_diff= -1;
	double diference=0; 
	double max_diference=0; 	
	if( translation_covariance.norm() > variance_limit_pos.norm() )
	{
	    for(unsigned int i = 0; i < points.size(); i++) 
	    {
		diference = fabs( mean.translation().norm() - points.at(i).translation().norm() );
		if( diference > max_diference )
		{
		    at_max_diff = i; 
		    max_diference = diference;
		}
	    }

	}
	else if( rotational_covariance(2,2) >  variance_limit_ori(2,2) ) 
	{
	    for(unsigned int i = 0; i < points.size(); i++) 
	    {
		diference = fabs( Matrix3d( points.at(i).rotation() ).eulerAngles(2,1,0)[0] - Matrix3d( mean.rotation() ).eulerAngles(2,1,0)[0] ); 
		if( diference > max_diference )
		{
		    at_max_diff = i; 
		    max_diference = diference;
		}
	    }
	    
	}

	if ( at_max_diff!=-1 ) 
	{
	    //cout << "Removing " << at_max_diff << " " << points.size()<< endl; 
	    //cout << "Removing " << points.at( at_max_diff).translation().transpose()<< endl; 
	    //outliers.push_back(points.at(  at_max_diff));
	    points.erase( points.begin() + at_max_diff); 
	    
	}else 
	{
	    //cout << "No point removed "<< endl; 
	    return true; 
	}
	
    }
    
    return false; 

}


/**
 * ******************************************************************
 * *****************************SAMPLING****************************
 */

Eigen::Affine3d Sampling::getSigmaOffset()
{
    return getSigmaSample(); 
}
Eigen::Affine3d Sampling::getUniformOffset()
{
    return getUniformSample(); 
}
	
 
void Sampling::defSearchRegion(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation)
{
    last_sigma_sample = 0;
    SigmaPoints sigma_points; 
    sigmaPointsPosition = sigma_points.calcSigmaPoints(cov_position, conf.region_sample_position);
    sigmaPointsOrientation = sigma_points.calcSigmaPoints(cov_orientation, conf.region_sample_orientation);
}

Eigen::Affine3d Sampling::getUniformSample( )
{
    Eigen::Vector3d translation; 
    translation.setZero(); 
    for( int col = 0; col < 3; col ++) 
    {
	translation = translation + getRandomValue(-1,1) * sigmaPointsPosition.col(col);

	
    }
    //removing z axis samplign 
    translation[2] = 0; 
    
    double delta_yaw = getRandomValue(-1,1)  * sigmaPointsOrientation.col(2).norm();
    
    Eigen::Affine3d offset( Eigen::AngleAxisd( delta_yaw, Eigen::Vector3d::UnitZ() ) );
    offset.translation() = translation; 
    
    //cout << offset.translation().transpose() << " yaw " << delta_yaw * 180 / M_PI << endl; 
 
    return offset;
    
}

Eigen::Affine3d  Sampling::getSigmaSample()
{
    if( last_sigma_sample == 0 ) 
	calcSigmaSamples(); 
    
    if( last_sigma_sample == sigma_samples.size() ) 
    {
	cout << " Icp.Stability.cpp Error: All sigma samples already collected " << endl; 
	return Eigen::Affine3d( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
    }
	
    last_sigma_sample++; 
    return sigma_samples.at(last_sigma_sample-1); 
    
}


void  Sampling::calcSigmaSamples()
{
    
    sigma_samples.clear();

    //the null sigma points 
    Eigen::Affine3d offset( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
    offset.translation() = Eigen::Vector3d::Zero(); 
    sigma_samples.push_back(offset);

    
    for(int i=0; i < 2; i++) 
    {
	int sign; 
	if ( i == 0) 
	    sign = 1;
	else 
	    sign = -1; 
	
	//Changed to column == 2 to ignore the z axis sampling 
	for(int column = 0; column < 2; column++) 
	{
	    Eigen::Affine3d offset( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
	    offset.translation() = Eigen::Vector3d( sign * sigmaPointsPosition.col(column) );
	    sigma_samples.push_back(offset);

	}
	
	Eigen::Affine3d offset( Eigen::AngleAxisd( sign * sigmaPointsOrientation.col(2).norm(), Eigen::Vector3d::UnitZ() ) );
	offset.translation() = Eigen::Vector3d::Zero();
	sigma_samples.push_back(offset);
	
    }    
    
}
	
	
/**
 * **************** HISTOGRAM****************************
 * *******************************************************
 */


double Histogram::calcSVMValue( ) 
{
    
    double bin_area = 0.13068;
    double f = 8.25721077416 * histogram.at(2) 
	    + 7.27432450174 * histogram.at(3)
	    + 0.901085498402 * histogram.at(4)
	    + 1.7194718866 * histogram.at(5)
	    + -5.12025789975 * histogram.at(6)
/*	    + -5.57942365201 * histogram.at(7)
	    + -3.69004685039 * histogram.at(8)*/
	    + -2.61875927499 * bin_area
	    + -6.33104159879;

    //std::cout << " Histogram Analysis F < -1 Bad F > 1 good " << f << std::endl; 
    return f; 
    
}

void Histogram::calculateHistogram( std::vector<double> pairs_distance ) 
{
    std::vector<double> pairs_distance_normalized; 
    pairs_distance_normalized.clear(); 
    
    for(size_t i=0;i<pairs_distance.size();i++) 
    {
	pairs_distance_normalized.push_back( (pairs_distance.at(i) - conf.mean)/conf.sigma); 
// 	std::cout << pairs_distance_normalized.at(i) << " " ; 
    }
//     std::cout << std::endl; 
    calculateStandartGaussianHistogram(pairs_distance_normalized);
//     for( size_t i =0; i < histogram.size(); i++) 
//       std::cout << histogram.at(i) << " " ; 
//      std::cout << std::endl; 
    svm_value = calcSVMValue(); 

}

void Histogram::calculateBinLimits( int n_sigma, int number_bins )
{

    histogram_limits.clear();
    
    //in a standart gaussian sigma = 1 
    double total_bin_area = n_sigma * 2; 
    double bin_area = total_bin_area / number_bins; 
    
    double lower_limit = -n_sigma; 
    
    for( int bin =0; bin <= number_bins; bin ++ )
	histogram_limits.push_back( lower_limit + bin_area*bin );
    
//     for( int bin =0; bin <= number_bins; bin ++ )
// 	std::cout << histogram_limits.at( bin ) << " "; 
//     std::cout << std::endl; 
}

void Histogram::calculateStandartGaussianHistogram(std::vector<double> pairs_distance)
{
    histogram.clear();	

    uint i = 0; 
    int number_of_points = 0; 
    int total_point = 0; 
    uint bin = 0; 
    uint number_bins = histogram_limits.size() - 1; 
    
    while (i < pairs_distance.size() && bin < number_bins) 
    {	  
	  if(  pairs_distance.at(i) < histogram_limits[0] ) 
	     i++; 
	  else if ( pairs_distance.at(i) > histogram_limits[bin] && pairs_distance.at(i) < histogram_limits[bin+1]) 
	  {
		number_of_points ++; 
		i++; 
	  }
	  else 
	  {
		total_point = total_point + number_of_points; 
		histogram.push_back( number_of_points); 
		bin ++; 
		number_of_points = 0; 
	  }
    }
    while(bin < number_bins) 
    {
	histogram.push_back(0);
	bin++;
    }
    
    //normalize 
    double bin_area = fabs(histogram_limits.at(1) -  histogram_limits.at(0));
    
    for(uint j = 0; j < histogram.size(); j++) 
	histogram.at(j) = histogram.at(j) / (bin_area * total_point); 
    
    
}

