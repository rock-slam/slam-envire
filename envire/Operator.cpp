#include "Core.hpp"

using namespace envire;

bool Operator::addInput( Layer* layer ) 
{
    env->addInput( this, layer );
}

bool Operator::addOutput( Layer* layer ) 
{
    env->addOutput( this, layer );
}

void Operator::removeInput( Layer* layer )
{
    env->removeInput( this, layer );
}

void Operator::removeOutput( Layer* layer )
{
    env->removeOutput( this, layer );
}

