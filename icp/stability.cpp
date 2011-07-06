#include "stability.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/SVD>
//#include <Eigen/Cholesky>
#include <Eigen/Dense>

USING_PART_OF_NAMESPACE_EIGEN

using namespace std; 
using namespace Eigen; 
using namespace envire::icp;


bool Clustering::cluster(  std::vector<Eigen::Transform3d> points ) 
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
	variance_postion = variance_postion + (mean.translation() - points.at(i).translation()).cwise().square(); 
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

Eigen::Transform3d Sampling::getSigmaOffset()
{
    return getSigmaSample(); 
}
Eigen::Transform3d Sampling::getUniformOffset()
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

Eigen::Transform3d Sampling::getUniformSample( )
{
    Eigen::Vector3d translation; 
    translation.setZero(); 
    for( int col = 0; col < 3; col ++) 
    {
	translation = translation + getRandomValue(-1,1) * sigmaPointsPosition.col(col);
	translation = translation + sigmaPointsPosition.col(col);
	
    }
    //removing z axis samplign 
    translation[2] = 0; 
    
    double delta_yaw = getRandomValue(-1,1)  * sigmaPointsOrientation.col(2).norm();
    
    Eigen::Transform3d offset( Eigen::AngleAxisd( delta_yaw, Eigen::Vector3d::UnitZ() ) );
    offset.translation() = translation; 
    
    //cout << offset.translation().transpose() << " yaw " << delta_yaw * 180 / M_PI << endl; 
 
    return offset;
    
}

Eigen::Transform3d  Sampling::getSigmaSample()
{
    if( last_sigma_sample == 0 ) 
	calcSigmaSamples(); 
    
    if( last_sigma_sample == sigma_samples.size() ) 
    {
	cout << " Icp.Stability.cpp Error: All sigma samples already collected " << endl; 
	return Eigen::Transform3d( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
    }	
	
    last_sigma_sample++; 
    return sigma_samples.at(last_sigma_sample-1); 
    
}


void  Sampling::calcSigmaSamples()
{
    
    sigma_samples.clear();

    //the null sigma points 
    Eigen::Transform3d offset( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
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
	    Eigen::Transform3d offset( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
	    offset.translation() = Eigen::Vector3d( sign * sigmaPointsPosition.col(column) );
	    sigma_samples.push_back(offset);

	}
	
	Eigen::Transform3d offset( Eigen::AngleAxisd( sign * sigmaPointsOrientation.col(2).norm(), Eigen::Vector3d::UnitZ() ) );
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
	    + -5.57942365201 * histogram.at(7)
	    + -3.69004685039 * histogram.at(8)
	    + -2.61875927499 * bin_area
	    + -6.33104159879;

    //std::cout << " Histogram Analysis F < -1 Bad F > 1 good " << f << std::endl; 
    return f; 
    
}

void Histogram::calculateHistogram( std::vector<double> pairs_distance ) 
{
    if ( conf.normalization ) 
	calculateNormalizedHistogram( pairs_distance ); 
    
    //else 
	//calculateNotNormalizedHistogram( pairs_distance ); 
	
    svm_value = calcSVMValue(); 

}

void Histogram::calculateNormalizedHistogram(std::vector<double> _pairs_distance)
{
    
    std::vector<double> pairs_distance; 
    pairs_distance.clear(); 
    
    for(size_t i=0;i<_pairs_distance.size();i++) 
    {
	pairs_distance.push_back( (_pairs_distance.at(i)- conf.mean)/conf.sigma); 
    }

    histogram.clear();
    histogram_limits.clear(); 

    int bin = 1; 
    
    double bin_size = conf.area / conf.number_bins; 

    int number_of_points = 0; 
    unsigned int i = 0; 

    //outlier will be stores at the first and last bin, so the total number of bins if bin_size + 2, so those bins will have variable size 
    //the frequency inside a bin is given by number of points inside a bin / ( total points * width of bin ) 

    //Calculate the bin placement (the central point is zero, since the data is normalized 
    
    //add the lower limit for the outliner 
    double outliner_upper_limit = (-conf.number_bins / 2)  * bin_size;
    
    if (  pairs_distance.front() < outliner_upper_limit ) 
	histogram_limits.push_back( outliner_upper_limit + pairs_distance.front() );
    else 
	histogram_limits.push_back( outliner_upper_limit - bin_size );
    
    
    double bin_lower_limit = 0; 
    for (bin = 0; bin <= conf.number_bins; bin++) 
    {
	bin_lower_limit = ((-conf.number_bins / 2) + bin ) * bin_size;
	histogram_limits.push_back( bin_lower_limit );
	
    }    
    
    //add the upper limit for the outliner 
    if ( pairs_distance.back() > bin_lower_limit) 
	histogram_limits.push_back( bin_lower_limit + pairs_distance.back() );
    else 
	histogram_limits.push_back( bin_lower_limit + bin_size );
    
    
    double bin_area = 0;
    //fill the bins 
    bin = 1; 
    while (i < pairs_distance.size()) 
    {
	  
	  if ( pairs_distance.at(i) < histogram_limits[bin]) 
	  {
		number_of_points ++; 
		i++; 
	  }
	  else 
	  {
		
		bin_area =fabs(histogram_limits[bin-1] - histogram_limits[bin]); 
		histogram.push_back( number_of_points/ ( pairs_distance.size() * conf.area) ); 
		
		bin ++; 
		//the +2 is for the outliers bins
		if (bin == conf.number_bins+2) 
		{
		    number_of_points = pairs_distance.size() - i; 
		    bin_area = fabs(histogram_limits[bin-1] - histogram_limits[bin]); 
		    histogram.push_back( number_of_points / (pairs_distance.size() * bin_area) );
		    return; 
		}
		  
		number_of_points = 0; 
	  }
    }

    bin_area = fabs(histogram_limits[bin-1] - histogram_limits[bin]); 
    histogram.push_back( number_of_points/ ( pairs_distance.size() * bin_area) ); 
    
   
    bin++;
    
    //the +2 is because of the 2 outliers bins, upper and lower
    while(bin <= (conf.number_bins+2)) 
    {
      histogram.push_back(0);
      bin++;
      
    }

}