#include "Featurecloud.hpp"

using namespace envire;
ENVIRONMENT_ITEM_DEF( Featurecloud )

Featurecloud::Featurecloud()
{
}

Featurecloud::Featurecloud(Serialization& so)
    : Pointcloud(so)
{
    // TODO read map file
}

void Featurecloud::serialize(Serialization& so)
{
    Pointcloud::serialize( so, false );

    // TODO write map file
}


void Featurecloud::clear() 
{
    Pointcloud::clear();
    keypoints.clear();
    descriptors.clear();
}

