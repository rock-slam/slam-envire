#include "icpLocalization.hpp"

#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <base/timemark.h>
#include <boost/concept_check.hpp>
#include <boost/scoped_ptr.hpp>
USING_PART_OF_NAMESPACE_EIGEN

using namespace std; 
using namespace envire::icp;


void ICPLocalization::initializePointCloud(ICPPointCloudConfiguration conf_point_cloud){
  
    this->conf_point_cloud = conf_point_cloud; 
    scansWithTransforms.clear();
        
    lastScanIndex = 0;
    scanCount = 0; 
}

void ICPLocalization::saveEnvironment()
{
    // write environment, if path is given
    if( !conf.environment_debug_path.empty() )
    {
	envire::Serialization so;
	so.serialize(env.get(), conf.environment_debug_path );
    }else
    {
	std::cout << "WARNING: IcpLocalization.cpp No path for saving the environment given" << std::endl; ; 
    }
}

void ICPLocalization::addLaserScan(Eigen::Transform3d body2Odo, Eigen::Transform3d body2World, Eigen::Transform3d laser2Body, const ::base::samples::LaserScan &scan_reading)
{
    LaserAndTransform lat;
    lat.scan = scan_reading;
    lat.body2Odo = body2Odo;
    lat.body2World = body2World;
    lat.laser2Body = laser2Body;
    scansWithTransforms.push_back(lat);
    while( scansWithTransforms.size() > conf_point_cloud.lines_per_point_cloud )
	scansWithTransforms.pop_front();
    
    scanCount++;
  
}

void ICPLocalization::addScanLineToPointCloud(Eigen::Transform3d body2Odo, Eigen::Transform3d body2World, Eigen::Transform3d laser2Body, const ::base::samples::LaserScan &scan_reading)
{

    if(scansWithTransforms.size() == 0) 
    {
	addLaserScan(body2Odo,body2World,laser2Body,scan_reading); 
	return;
    }
    
    double max_rotation =  1.5 * conf_point_cloud.lines_per_point_cloud * conf_point_cloud.min_rotation_for_new_line;
    double max_translation = 1.5 * conf_point_cloud.lines_per_point_cloud * conf_point_cloud.min_distance_travelled_for_new_line;
    double max_head_movement = 1.5 * conf_point_cloud.lines_per_point_cloud * conf_point_cloud.min_rotation_head_for_new_line; 
    bool add_laser_scan = true; 
    for( uint i = 0; i < scansWithTransforms.size(); i++) 
    {
	Eigen::Transform3d diference( body2Odo.inverse() * scansWithTransforms.at(i).body2Odo );

	Vector3d Ylaser2Body = laser2Body * Vector3d::UnitY() - laser2Body.translation();
	Ylaser2Body.normalize();
	Vector3d YlastLaser2Body = scansWithTransforms.back().laser2Body * Vector3d::UnitY() - scansWithTransforms.at(i).laser2Body.translation();
	YlastLaser2Body.normalize();
	
	double laserChange = acos(Ylaser2Body.dot(YlastLaser2Body));
	double translation =  diference.translation().norm(); 
	double rotation = fabs(Eigen::AngleAxisd( diference.rotation() ).angle()) ; 
	
	add_laser_scan = add_laser_scan && ( rotation > conf_point_cloud.min_rotation_for_new_line || translation > conf_point_cloud.min_distance_travelled_for_new_line || laserChange >  conf_point_cloud.min_rotation_head_for_new_line);
	 
	//if  the distance is to big means the old laser scan is not consistent anymore. 
	if( rotation > max_rotation|| translation > max_translation || laserChange > max_head_movement)
	{
	    scansWithTransforms.erase(scansWithTransforms.begin() + i); 
	    scanCount--; 
	    std::cout << " IcpLocalization.cpp erasing old laser scan " << std::endl; 
	}

/*	std::cout <<" add new scan " << add_laser_scan << std::endl; 
	std::cout << "\t translation " << (translation > conf_point_cloud.min_distance_travelled_for_new_line)<< " "  << translation << " > " << conf_point_cloud.min_distance_travelled_for_new_line << std::endl;
	std::cout << "\t rotation " << (rotation > conf_point_cloud.min_rotation_for_new_line) << " "  << rotation * 180 / M_PI << " > " << conf_point_cloud.min_rotation_for_new_line * 180 / M_PI << std::endl;
	std::cout << "\t head " << (laserChange >  conf_point_cloud.min_rotation_head_for_new_line) << "  "<< laserChange * 180 / M_PI << " > " << conf_point_cloud.min_rotation_head_for_new_line * 180 / M_PI<< std::endl; */
	
	if (!add_laser_scan) 
	    break; 
    }
    
    if ( add_laser_scan )
    {
	addLaserScan(body2Odo,body2World,laser2Body,scan_reading); 
    }
    
    return; 
  
}

bool ICPLocalization::hasNewPointCloud()
{
    if( scanCount > conf_point_cloud.lines_per_point_cloud && (scanCount - lastScanIndex) > conf_point_cloud.min_line_advance )
    { 	
	return true;
    } 
    
    return false; 
}

void ICPLocalization::loadEnvironment(ICPModelConfiguration conf)
{
    std::cout << " Loading Environment "<<conf.environment_path << " with density " << conf.model_density << std::endl; 
    if ( conf.environment_path != "" ) 
    {
	// load the environment
	envire::Serialization so;
	boost::scoped_ptr<envire::Environment>(so.unserialize( conf.environment_path) ).swap( env );
    }
    
    // and load all the pointcloud data into the icp model
    std::vector<envire::Pointcloud*> items = env->getItems<envire::Pointcloud>();
    for(std::vector<envire::Pointcloud*>::iterator it=items.begin();it!=items.end();it++)
    {
	icp.addToModel( envire::icp::PointcloudAdapter( *it, conf.model_density ) );
	std::cout << "adding model to icp. using density " << conf.model_density << std::endl;
    }
}

ICPInputData ICPLocalization::generatePointcloud()
{
    base::Time startTime = base::Time::now();
    envire::Pointcloud *pc = new envire::Pointcloud();

    Eigen::Transform3d odo2World;
    base::Time pointCloudTime; 
    base::Time lastLSTime; 

    {
	
	const LaserAndTransform &lastScan = scansWithTransforms.back();
	curBody2World = lastScan.body2World;
	odo2World = curBody2World * lastScan.body2Odo.inverse();
	pointCloudTime = lastScan.scan.time;

	for(std::deque<LaserAndTransform >::const_iterator it = scansWithTransforms.begin(); it != scansWithTransforms.end(); it++) {
	    // transform in vector is body2Odo
	    //(odo2World * it->transform) == body2World
	    const Eigen::Transform3d laser2World( (odo2World * it->body2Odo)  * it->laser2Body );
	    const Eigen::Transform3d laser2CurBody( curBody2World.inverse() * laser2World );

	    std::vector<Eigen::Vector3d> line = it->scan.convertScanToPointCloud( laser2CurBody ); 
	    std::copy( line.begin(), line.end(), std::back_inserter( pc->vertices ) );
	}
    }
  
    ICPInputData newData;
    newData.pc2World = curBody2World; //Interpolated point being corrected 
    newData.pc = pc;
    newData.pointCloudTime = pointCloudTime;
    
    //base::Time curTime = base::Time::now();
    //std::cout << "Time of PC generation " << curTime << " pcgenTime " << curTime-startTime<< " pc time " << pointCloudTime << " diff " << curTime -  pointCloudTime << std::endl;

    lastScanIndex = scanCount;
    
    return newData;
}

ICPInputData ICPLocalization::generatePointcloudSample(ICPInputData originalData, Eigen::Transform3d offset)
{
    ICPInputData newData;

    newData.pc2World = offset * originalData.pc2World; 
    newData.pc2World.translation() = offset.translation() + originalData.pc2World.translation();
    newData.pc = originalData.pc->clone(); 
    newData.pointCloudTime = originalData.pointCloudTime;
    
    return newData; 
}

void ICPLocalization::removeLastSavedPointCloud()
{
    env->detachItem( pc );
    env->detachItem( fn );	
}
ICPResult ICPLocalization::doScanMatch(struct ICPInputData& inputData, bool save)
{
    
    pc = inputData.pc;
    env->attachItem( pc );
    
    fn = new envire::FrameNode( inputData.pc2World);
    env->attachItem( fn );
    
    env->addChild( env->getRootNode(), fn ); 
    pc->setFrameNode( fn );

    inputData.fn = fn;
    
    // run the icp
    base::TimeMark m1("icp");
    icp.align( envire::icp::PointcloudAdapter( pc, conf.measurement_density ),  conf.max_iterations, conf.min_mse, conf.min_mse_diff, conf.overlap );
    
    ICPResult result;
    
    result.time = inputData.pointCloudTime; 
    result.points = pc->vertices.size(); 
    result.from = inputData.pc2World; 
    result.pairs = icp.getPairs(); 
    
    if(result.pairs > 0) {
	
	result.mse = icp.getMeanSquareError(); 
	result.to = fn->getTransform(); 
	result.pairs_distance = icp.getPairsDistance(); 

	switch(conf.cov_conf.mode){ 
	    case HARD_CODED: 
	    {
		result.cov_position = conf.cov_conf.cov_position; 
		result.cov_orientation = conf.cov_conf.cov_orientation; 
		break; 
	    }
	    case MSE_BASED: 
	    {
		  float avgDist = std::max( 1.0, conf.measurement_density )/4.0;
		  float mseFactor = avgDist/sqrt(icp.getMeanSquareError());
		  result.cov_position = Eigen::Matrix3d::Identity() * (1e-3* 2.0/ pow(mseFactor*.5,4)); 
		  result.cov_orientation = Eigen::Matrix3d::Identity() *( 1.0 * M_PI / 180 )/ pow(mseFactor*.5,4) ; 	  
		break; 
	    }
	    default: 
	    {
		result.cov_position = Eigen::Matrix3d::Ones()*INFINITY; 
		result.cov_orientation = Eigen::Matrix3d::Ones()*INFINITY; 
		break; 
	    }
	}

    }
    
  
    if(!save || conf.environment_debug_path.empty())
    {
	env->detachItem( pc );
	env->detachItem( fn );	
    }
    
    return result; 
}