#include "stability.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

USING_PART_OF_NAMESPACE_EIGEN

using namespace std; 
using namespace Eigen; 
using namespace envire::icp;


void Clustering::calcVariance( )
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
    
    cout << " variance " << endl; 
    cout << variance_postion.transpose() << " - " << variance_yaw * 180 / M_PI << endl; 
    
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
    std::cout <<points.size() <<" mean " << position_mean.transpose() << " - " <<  yaw_mean * 180 / M_PI <<  endl; 
    mean = Eigen::AngleAxisd( yaw_mean, Eigen::Vector3d::UnitZ() ); 
    mean.translation() = position_mean;

}

void Clustering::calcSpread(Eigen::Matrix3d cov_position, Eigen::Matrix3d cov_orientation, double percentage_sigma, double min_distance, double min_angle, double max_distance, double max_angle)
{
    //calculating the square of the covariance 
    Eigen::LLT<Eigen::Matrix3d> llt;
    llt.compute(cov_position);
    Matrix3d sigma_points = llt.matrixL();
    llt.compute(cov_orientation);
    Matrix3d sigma_points_ori = llt.matrixL();    

    spread = Vector4d(sigma_points.col(0).norm(), sigma_points.col(1).norm(), sigma_points.col(2).norm(), sigma_points_ori.col(2).norm() );

    spread = percentage_sigma * spread;
    
    for(int i = 0; i < 3; i++) 
    {
	if ( spread(i) < min_distance )
	    spread(i) = min_distance; 
	else if ( spread(i) > max_distance ) 
	    spread(i) = max_distance; 
    }
    
    if ( spread(3) < min_angle )
	spread(3) = min_angle; 
    else if ( spread(3) > max_angle ) 
	spread(3) = max_angle; 
 
}

void Clustering::removeOutliners( int clustering_min_points )
{
    outliners.clear();
    cout << "spread " << spread.segment<3>(0).transpose() << " " <<spread(3) * 180 / M_PI<< endl; 
    while( points.size() >= clustering_min_points )
    {
	calcMean( ); 
	
	//verify if all points are within the clustering distance limits 
	int at_max_diff= -1;
	
	Eigen::Vector4d diference; 
	Eigen::Vector4d max_diference; 
	
	max_diference.setZero(); 
	
	for(unsigned int i = 0; i < points.size(); i++) 
	{
	    diference.segment<3>(0) = ( mean.translation() - points.at(i).translation() ).cwise().abs(); 
	    diference(3) = fabs( Matrix3d( points.at(i).rotation() ).eulerAngles(2,1,0)[0] - Matrix3d( mean.rotation() ).eulerAngles(2,1,0)[0] ); 
	    for(int i = 0; i < 4; i++) 
		if ( diference(i) > spread(i) ) 
		{
		    if ( diference.norm() > max_diference.norm() )
		    {
			at_max_diff = i; 
			max_diference = diference;
			break; 
		    }
		}
   	}
	
	if ( at_max_diff!=-1 ) 
	{
	    cout << "Removing " << points.at( at_max_diff).translation().transpose()<< endl; 
	    outliners.push_back(points.at(  at_max_diff));
	    points.erase( points.begin() + at_max_diff); 
	    
	}else 
	{
	    cout << "No point removed "<< endl; 
	    return; 
	}
    }
    
    return; 

}

/**
 * ******************************************************************
 * *****************************SAMPLING****************************
 */

Eigen::Transform3d Sampling::getOffset()
{
    
    switch (sampling_type)
    {
	case 0: 
	    return getZeroSample(); 
	case 1: 
	    return getSigmaSample(); 
	case 2: 
	    return getUniformSample();
	default: 
	    cout << "ERROR STABILITY.CPP NO SAMPLING MODE DEFINED " << endl; 
    }    
    return  Eigen::Transform3d( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) ) ; 
    
}
	
Eigen::Transform3d Sampling::getZeroSample()
{
    
    Eigen::Transform3d offset( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
    return offset; 
    
}

Eigen::Transform3d Sampling::getUniformSample( )
{
    
    Eigen::Vector3d translation; 
    translation.setZero(); 
    for( int col = 0; col < 3; col ++) 
    {
	translation = translation + (*generator)()* sigmaPointsPosition.col(col);
	
    }
    double delta_yaw = (*generator) () * sigmaPointsOrientation.col(2).norm();
    Eigen::Transform3d offset( Eigen::AngleAxisd( delta_yaw, Eigen::Vector3d::UnitZ() ) );
    offset.translation() = translation; 
    
    cout << offset.translation().transpose() << " yaw " << delta_yaw * 180 / M_PI << endl; 

    return offset;
    
}

Eigen::Transform3d  Sampling::getSigmaSample()
{
    
    if( last_sigma_sample == sigma_samples.size() ) 
    {
	cout << " Icp.Stability.cpp Error: All sigma samples already collected " << endl; 
	return Eigen::Transform3d( Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) );
    }
	
	
    last_sigma_sample++; 
    return sigma_samples.at(last_sigma_sample-1); 
    
}

void  Sampling::calcSigmaPoints(Eigen::Matrix3d cov_pos, Eigen::Matrix3d cov_or, double min_distance, double min_angle)
{
    
    //calculating the square of the covariance 
    Eigen::LLT<Eigen::Matrix3d> llt;
    llt.compute(cov_pos);
    sigmaPointsPosition = llt.matrixL();    

    llt.compute(cov_or);
    sigmaPointsOrientation = llt.matrixL();    

    for(int column = 0; column < 3; column++) 
    {
	
	if( sigmaPointsPosition.col(column).norm() < min_distance ) 
		sigmaPointsPosition.col(column) = (Eigen::Matrix3d::Identity() * min_distance).col(column) ;
	if( sigmaPointsOrientation.col(column).norm() < min_angle ) 
		sigmaPointsOrientation.col(column) = (Eigen::Matrix3d::Identity() * min_angle).col(column) ; 
    }
    
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
	
	for(int column = 0; column < 3; column++) 
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


double Histogram::gethistogramSVNClassification( ) 
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

void Histogram::calculateHistogram(std::vector<double> pairs_distance)
{
    
    if ( normalization ) 
	calculateNormalizedHistogram( pairs_distance ); 
    //else 
	//calculateNotNormalizedHistogram( pairs_distance ); 
	
}

void Histogram::calculateNormalizedHistogram(std::vector<double> _pairs_distance)
{
    
    std::vector<double> pairs_distance; 
    pairs_distance.clear(); 
    
    for(size_t i=0;i<_pairs_distance.size();i++) 
    {
	pairs_distance.push_back( (_pairs_distance.at(i)- mean)/sigma); 
    }

    histogram.clear();
    histogram_limits.clear(); 

    int bin = 1; 
    
    double bin_size = area / number_bins; 

    int number_of_points = 0; 
    unsigned int i = 0; 

    //outlier will be stores at the first and last bin, so the total number of bins if bin_size + 2, so those bins will have variable size 
    //the frequency inside a bin is given by number of points inside a bin / ( total points * width of bin ) 

    //Calculate the bin placement (the central point is zero, since the data is normalized 
    
    //add the lower limit for the outliner 
    double outliner_upper_limit = (-number_bins / 2)  * bin_size;
    
    if (  pairs_distance.front() < outliner_upper_limit ) 
	histogram_limits.push_back( outliner_upper_limit + pairs_distance.front() );
    else 
	histogram_limits.push_back( outliner_upper_limit - bin_size );
    
    
    double bin_lower_limit = 0; 
    for (bin = 0; bin <= number_bins; bin++) 
    {
	bin_lower_limit = ((-number_bins / 2) + bin ) * bin_size;
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
		histogram.push_back( number_of_points/ ( pairs_distance.size() * area) ); 
		
		bin ++; 
		//the +2 is for the outliners bins
		if (bin == number_bins+2) 
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
    
    //the +2 is because of the 2 outliners bins, upper and lower
    while(bin <= (number_bins+2)) 
    {
      histogram.push_back(0);
      bin++;
      
    }

}