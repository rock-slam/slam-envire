#ifndef __ICP_CONFIGURATION_TYPES__
#define __ICP_CONFIGURATION_TYPES__

#include <string> 
#include <boost/concept_check.hpp>
#include <base/eigen.h>

namespace envire {
namespace icp
{

    enum COV_MODE {  
	HARD_CODED,  
	MSE_BASED,
	NO_COV
    };
    
    /** 
     * configure the icp result covariance 
     */ 
    struct ICPResultCovarianceConf
    {
	COV_MODE mode;
	base::Matrix3d cov_position; 
	base::Matrix3d cov_orientation;
	
	ICPResultCovarianceConf() 
	    : mode(HARD_CODED) {}
    }; 
    
    /**
     * Configures the model to be loaded
     */
    struct ICPModelConfiguration{
      	/** Path to the model to be loaded */
	std::string environment_path; 
	/** The denisity of the model that should be used */ 
	double model_density; 
    }; 
    
    /** 
     * Configuration on how to construct a point cloud 
     */ 
    struct ICPPointCloudConfiguration{
      	/**'minimal number of scanlines for creating a point cloud and triggerign an icp */
	int lines_per_point_cloud;
	/** minimum number of lines that dont overlap with the previous scan */
	int min_line_advance; 
	/**minimum distance travelled to trigger a new line'*/
	double min_distance_travelled_for_new_line; 
	/**'minimum angle travelled to trigger a new line'*/
	double min_rotation_for_new_line; 
	/**'minimum angular rotation on head to trigger a new line'*/
	double min_rotation_head_for_new_line; 
    };
    
    /**
     * icp.hpp class configuration 
     */
    struct ICPConfiguration
    {  


	/** maximum number of iterations for the icp algorithm */
	int max_iterations;
	/**overlap between model and measurement (between [0..1])*/
	double overlap; 
	/**icp will stop if the mean square error is smaller than the given value.'*/
	double min_mse;
	/**icp will stop if the difference in mean square error is smaller*/
	double min_mse_diff; 
	/**'density of the measurement pointcloud*/
	double measurement_density;
	
	ICPResultCovarianceConf cov_conf; 
	
      	/**if this property set, scans will be collected, and environment written to given path when the module stops*/
	std::string environment_debug_path; 
	
    };

    /**
     *Defines the limits of the sigma points  
     */
    struct SigmaPointConfiguration 
    {
	/** the minimal value of the sigma point norm  */ 
	double min_norm; 
	/** the maximal value of the sigma point norm  */ 
	double max_norm; 
	/** a multiplier for the sigma point  */ 
	int n_sigma; 
    }; 
    
    /**
     * Configures the stability.hpp cluster class 
     */ 
    struct ClusteringConfiguration{

	/**the minimal number of points needed in a cluster */
	uint min_number_of_points; 
	
	bool remove_outliers; 
	
	/**defines the limits for removing the outliners in terms of position */
	SigmaPointConfiguration outliers_position;
	
	/**defines the limits for removing the outliners in terms of orientation */
	SigmaPointConfiguration outliers_orientation;
	
    }; 
    
    enum SAMPLING_MODE {  
	SIGMA_SAMPLING, 
	UNIFORM_SAMPLING
    }; 
    /**
     * Configuration class for the sampling.hpp class 
     */
    struct SamplingConfiguration{
	
	SAMPLING_MODE mode; 
	
	/** defines the limits of the sampling region in terms of position  */ 
	SigmaPointConfiguration region_sample_position;
	/** defines sampling region in terms of rotation  */ 
	SigmaPointConfiguration region_sample_orientation;
	
	SamplingConfiguration() 
		: mode(UNIFORM_SAMPLING) {}
	
    }; 
    
    /** 
     * Configuration for the histogram.hpp clss 
     */ 
    //TODO FIX THE CONFIGURATION ! 
    struct HistogramConfiguration{
	/** Threshold for hte histogram rejection */ 
	double histogram_rejection_threshold; 
	/** Number of bins in the histogram */ 
	double number_bins; 
	/** total area of the histogram */ 
	double area;
	/** if the histogram should be normalized */
	bool normalization; 
	/** if 2 extra bins should be added for the outliners*/ 
	bool outliners;
	/** mean of the normalization */ 
	double mean; 
	/** sigma for the normalization */ 
	double sigma; 
    };



}
}
#endif


