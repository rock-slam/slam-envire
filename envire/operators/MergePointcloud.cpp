/*
 * MergePointcloud.cpp
 *
 *  Created on: 14.04.2010
 *      Author: planthaber
 */

#include "MergePointcloud.hpp"
#include <Eigen/LU>

namespace envire {

const std::string MergePointcloud::className = "envire::MergePointcloud";

MergePointcloud::MergePointcloud() {
    // TODO Auto-generated constructor stub

}

MergePointcloud::MergePointcloud(Serialization& so):Operator(so){
    so.setClassName(className);
}


MergePointcloud::~MergePointcloud() {
    // TODO Auto-generated destructor stub
}


void MergePointcloud::serialize(Serialization& so){
    Operator::serialize(so);
    so.setClassName(className);
}

void MergePointcloud::addInput(Pointcloud* pc){
    Operator::addInput(pc);
}

void MergePointcloud::addOutput(Pointcloud* globalpc){
    if( env->getOutputs(this).size() > 0 )
	throw std::runtime_error("MergePointcloud can only have one output.");

    Operator::addOutput(globalpc);
}

bool MergePointcloud::updateAll(){
    Pointcloud* targetcloud = dynamic_cast<envire::Pointcloud*>(*env->getOutputs(this).begin());
    assert( targetcloud );
    targetcloud->vertices.clear();

    std::list<Layer*> inputs = env->getInputs(this);

    // check for additional data 
    bool hasNormal = false, hasColor = false;

    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ ){
	Pointcloud* cloud = dynamic_cast<envire::Pointcloud*>(*it);
	hasNormal = hasNormal || cloud->hasData( Pointcloud::VERTEX_NORMAL );
	hasColor = hasColor || cloud->hasData( Pointcloud::VERTEX_COLOR );
    }

    //for every cloud
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ ){
	Pointcloud* cloud = dynamic_cast<envire::Pointcloud*>(*it);
	assert(cloud);

	FrameNode::TransformType trans = 
	    env->relativeTransform( cloud->getFrameNode(), targetcloud->getFrameNode() );


	for (std::vector<Eigen::Vector3d>::iterator p = cloud->vertices.begin();p<cloud->vertices.end();p++)
	{
	    targetcloud->vertices.push_back(trans * *p);
	}

	if( hasNormal )
	{
	    if( !cloud->hasData( Pointcloud::VERTEX_NORMAL ) )
		throw std::runtime_error("merge currently needs to have the same metadata on all inputs");

	    Eigen::Quaterniond rot(trans.rotation());

	    std::vector<Eigen::Vector3d> &source_data( cloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL ) );
	    std::vector<Eigen::Vector3d> &target_data( targetcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL ) );

	    std::cout << source_data.size() << std::endl;
	    for (std::vector<Eigen::Vector3d>::iterator p = source_data.begin();p!=source_data.end();p++)
	    {
		target_data.push_back( rot * *p );
	    }
	}

	if( hasColor )
	{
	    if( !cloud->hasData( Pointcloud::VERTEX_COLOR ) )
		throw std::runtime_error("merge currently needs to have the same metadata on all inputs");

	    std::vector<Eigen::Vector3d> &source_data( cloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR ) );
	    std::vector<Eigen::Vector3d> &target_data( targetcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR ) );

	    for (std::vector<Eigen::Vector3d>::iterator p = source_data.begin();p!=source_data.end();p++)
	    {
		target_data.push_back( *p );
	    }
	}
    }

    return true;
}

}//namespace
