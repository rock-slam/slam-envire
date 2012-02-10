#include "Featurecloud.hpp"

using namespace envire;
ENVIRONMENT_ITEM_DEF( Featurecloud )

Featurecloud::Featurecloud()
    : descriptorSize(0)
{
}

void Featurecloud::serialize(Serialization& so)
{
    Pointcloud::serialize( so, false );

    // TODO write map file
}

void Featurecloud::unserialize(Serialization& so)
{
    Pointcloud::unserialize(so);
    
    // TODO read map file
}

void Featurecloud::clear() 
{
    Pointcloud::clear();

    keypoints.clear();
    descriptors.clear();
}

void Featurecloud::copyFrom( Featurecloud* source, bool transform )
{
    Pointcloud::copyFrom( source, transform );

    descriptorType = source->descriptorType;
    descriptorSize = source->descriptorSize;
    keypoints = source->keypoints;
    descriptors = source->descriptors;
}
