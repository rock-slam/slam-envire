#include "CutPointcloud.hpp"

namespace envire {

ENVIRONMENT_ITEM_DEF( CutPointcloud )

CutPointcloud::CutPointcloud() 
    : Operator(1,1)
{
    vertex_data_source.reset();
}

CutPointcloud::~CutPointcloud() 
{
}

void CutPointcloud::serialize(Serialization& so){
    Operator::serialize(so);
}

void CutPointcloud::unserialize(Serialization& so)
{
    Operator::unserialize(so);
}

void CutPointcloud::addInput(Pointcloud* pc){
    if( env->getInputs(this).size() > 0 )
        throw std::runtime_error("CutPointcloud can only have one input.");
        
    Operator::addInput(pc);
}

void CutPointcloud::addOutput(Pointcloud* pc){
    if( env->getOutputs(this).size() > 0 )
	throw std::runtime_error("CutPointcloud can only have one output.");

    Operator::addOutput(pc);
}

void CutPointcloud::addBox(ExclusionBox* box)
{
    std::list<ExclusionBox*>::iterator it = std::find(exclusion_boxes.begin(), exclusion_boxes.end(), box);
    if(it == exclusion_boxes.end())
        exclusion_boxes.push_back(box);
}

void CutPointcloud::removeBox(ExclusionBox* box)
{
    std::list<ExclusionBox*>::iterator it = std::find(exclusion_boxes.begin(), exclusion_boxes.end(), box);
    if(it != exclusion_boxes.end())
        exclusion_boxes.erase(it);
}

bool CutPointcloud::isIncluded(const Eigen::Vector3d &vector)
{
    for(std::list<ExclusionBox*>::const_iterator it = exclusion_boxes.begin(); it != exclusion_boxes.end(); it++)
    {
        if((*it)->box.contains(vector))
        {
            if((*it)->excludes())
                return false;
        }
        else
        {
            if((*it)->includes())
                return false;
        }
    }
    return true;
}

void CutPointcloud::copyVertexData(Pointcloud* sourcecloud, Pointcloud* targetcloud, bool do_transform, bool filter)
{
    targetcloud->clear();
    
    // get meta data
    std::vector<Eigen::Vector3d> *source_vertex_normal_data = NULL;
    std::vector<Eigen::Vector3d> *source_vertex_color_data = NULL;
    std::vector<Pointcloud::attr_flag> *source_vertex_attributes_data = NULL;
    std::vector<double> *source_vertex_variance_data = NULL;
    std::vector<Eigen::Vector3d> *target_vertex_normal_data = NULL;
    std::vector<Eigen::Vector3d> *target_vertex_color_data = NULL;
    std::vector<Pointcloud::attr_flag> *target_vertex_attributes_data = NULL;
    std::vector<double> *target_vertex_variance_data = NULL;
    if( sourcecloud->hasData(Pointcloud::VERTEX_NORMAL) )
    {
        source_vertex_normal_data = &sourcecloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL );
        target_vertex_normal_data = &targetcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL );
    }
    if( sourcecloud->hasData(Pointcloud::VERTEX_COLOR) )
    {
        source_vertex_color_data = &sourcecloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR );
        target_vertex_color_data = &targetcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR );
    }
    if( sourcecloud->hasData(Pointcloud::VERTEX_ATTRIBUTES) )
    {
        source_vertex_attributes_data = &sourcecloud->getVertexData<Pointcloud::attr_flag>( Pointcloud::VERTEX_ATTRIBUTES );
        target_vertex_attributes_data = &targetcloud->getVertexData<Pointcloud::attr_flag>( Pointcloud::VERTEX_ATTRIBUTES );
    }
    if( sourcecloud->hasData(Pointcloud::VERTEX_VARIANCE) )
    {
        source_vertex_variance_data = &sourcecloud->getVertexData<double>( Pointcloud::VERTEX_VARIANCE );
        target_vertex_variance_data = &targetcloud->getVertexData<double>( Pointcloud::VERTEX_VARIANCE );
    }
    
    // get transformation
    FrameNode::TransformType trans;
    Eigen::Quaterniond normal_rot;
    if(do_transform)
    {
        trans = env->relativeTransform( sourcecloud->getFrameNode(), targetcloud->getFrameNode() );
        normal_rot = trans.linear();
    }
    
    for (unsigned i = 0; i < sourcecloud->vertices.size(); i++)
    {
        if(!filter || isIncluded(sourcecloud->vertices[i]))
        {
            if(do_transform)
            {
                targetcloud->vertices.push_back(trans * sourcecloud->vertices[i]);
                
                if(source_vertex_normal_data && source_vertex_normal_data->size() > i)
                    target_vertex_normal_data->push_back( normal_rot * source_vertex_normal_data->at(i) );
            }
            else
            {
                targetcloud->vertices.push_back(sourcecloud->vertices[i]);
                
                if(source_vertex_normal_data && source_vertex_normal_data->size() > i)
                    target_vertex_normal_data->push_back(source_vertex_normal_data->at(i) );
            }
            
            if(source_vertex_color_data && source_vertex_color_data->size() > i)
                target_vertex_color_data->push_back( source_vertex_color_data->at(i) );
            
            if(source_vertex_attributes_data && source_vertex_variance_data->size() > i)
                target_vertex_attributes_data->push_back( source_vertex_attributes_data->at(i) );
            
            if(source_vertex_variance_data && source_vertex_variance_data->size() > i)
                target_vertex_variance_data->push_back( source_vertex_variance_data->at(i) );
        }
    }
}

bool CutPointcloud::updateAll()
{    
    Pointcloud* sourcecloud = dynamic_cast<envire::Pointcloud*>(env->getInputs(this).front());
    Pointcloud* targetcloud = dynamic_cast<envire::Pointcloud*>(env->getOutputs(this).front());
    assert( targetcloud && sourcecloud );
    
    bool do_transform = true;
    if(targetcloud == sourcecloud)
    {
        do_transform = false;
        if(!vertex_data_source.get())
        {
            vertex_data_source.reset(new Pointcloud);
            copyVertexData(sourcecloud, vertex_data_source.get(), do_transform, false);
        }
        sourcecloud = vertex_data_source.get();
    }
    
    copyVertexData(sourcecloud, targetcloud, do_transform);

    env->itemModified( targetcloud );
    return true;
}

}//namespace
