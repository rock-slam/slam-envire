#ifndef __ICP_LOCALIZATION_H__
#define __ICP_LOCALIZATION_H_

#define EIGEN_USE_NEW_STDVECTOR

#include<Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <boost/concept_check.hpp>
#include <boost/scoped_ptr.hpp>
#include <math.h>

#include "icp.hpp"
#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>

#include "icpConfigurationTypes.hpp"

#include <base/TimeMark.hpp>
#include <base/samples/LaserScan.hpp>
#include <deque>
namespace envire {
namespace icp {
  
//TODO ASK ABOUT DELETION OF THESE POINTERS!!!   
class ICPInputData {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      base::Time pointCloudTime;
      envire::Pointcloud* pc;
      envire::FrameNode* fn;
      Eigen::Affine3d pc2World;

};

struct ICPResult { 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    base::Time time; 
    int points;
    int pairs;
    double mse; 
    
    Eigen::Affine3d from; 
    Eigen::Affine3d to; 
    std::vector<double> pairs_distance; 
    
    Eigen::Matrix3d cov_position;
    Eigen::Matrix3d cov_orientation; 
    
};

class LaserAndTransform {
    public:  
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Affine3d body2Odo;
	Eigen::Affine3d body2World;
	Eigen::Affine3d laser2Body;
	base::samples::LaserScan scan;
};

class ICPLocalization
{
    private: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int lastScanIndex; 

	base::Time lastICPTime; 
	
	Eigen::Affine3d lastBody2Odo;
	
	Eigen::Affine3d lastLaser2Body;
	
	Eigen::Affine3d curBody2World;
	
	boost::scoped_ptr<envire::Environment> env;

	envire::icp::TrimmedKD icp;
	
	ICPConfiguration conf; 
	
	// the nunmber of scans added 
	int scanCount;
		 
	std::deque<LaserAndTransform, Eigen::aligned_allocator<LaserAndTransform> > scansWithTransforms;
	
	void addLaserScan(Eigen::Affine3d body2Odo, Eigen::Affine3d body2World, Eigen::Affine3d laser2Body, const ::base::samples::LaserScan &scan_reading);
	
	ICPPointCloudConfiguration conf_point_cloud;
	
	envire::Pointcloud *pc; 
	
	envire::FrameNode *fn;
    public: 
  
	void removeLastSavedPointCloud();
	
	void loadIcpConfiguration(ICPConfiguration conf){ this->conf = conf; }  
	
	void loadEnvironment(std::string environment_path, double model_density); 
	
	/** 
	* Copy a original point cloud offseting its original position 
	*/ 
	ICPInputData generatePointcloudSample(ICPInputData originalData, Eigen::Affine3d offset);
	/** 
	* Realizes an icp on a given point cloud 
	*/ 
	ICPResult doScanMatch(struct ICPInputData& inputData, bool save);
	
	void addScanLineToPointCloud(Eigen::Affine3d body2Odo, Eigen::Affine3d body2World, Eigen::Affine3d laser2Body, const ::base::samples::LaserScan &scan_reading); 
	
	void saveEnvironment(); 

	ICPInputData generatePointcloud();
	
	void initializePointCloud(ICPPointCloudConfiguration conf_point_cloud); 
	
	/**
	 * Return if there are enough scans to generate a new point cloud 
	 */
	bool hasNewPointCloud();
 
}; 
}
}
#endif 
