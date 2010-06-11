/*
 * MergePointcloud.cpp
 *
 *  Created on: 14.04.2010
 *      Author: planthaber
 */

#include "MergePointcloud.hpp"

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
    Pointcloud* targetcloud = static_cast<envire::Pointcloud*>(*env->getOutputs(this).begin());
    std::list<Layer*> inputs = env->getInputs(this);

    targetcloud->vertices.clear();

    //for every cloud
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ ){
	Pointcloud* cloud = dynamic_cast<envire::Pointcloud*>(*it);

	FrameNode::TransformType trans;
	trans.setIdentity();
	FrameNode* curr = env->getFrameNode(cloud);
	while (!curr->isRoot()){
	    trans = trans * curr->getTransform();
	    curr = curr->getParent();
	}



	//add all points


	//targetcloud->vertices.insert(targetcloud->vertices.end(),cloud->vertices.begin(),cloud->vertices.end());
	for (std::vector<Eigen::Vector3d>::iterator p = cloud->vertices.begin();p<cloud->vertices.end();p++){
	    targetcloud->vertices.push_back(trans * *p);
	}


    }
}

}//namespace
