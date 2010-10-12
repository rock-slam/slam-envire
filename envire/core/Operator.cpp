#include "Core.hpp"

using namespace envire;

const std::string Operator::className = "envire::Operator";

Operator::Operator()
{
}

Operator::Operator(Serialization& so)
    : EnvironmentItem(so)
{
}

bool Operator::addInput( Layer* layer ) 
{
    env->addInput( this, layer );
    return true;
}

bool Operator::addOutput( Layer* layer ) 
{
    env->addOutput( this, layer );
    return true;
}

void Operator::removeInput( Layer* layer )
{
    env->removeInput( this, layer );
}

void Operator::removeOutput( Layer* layer )
{
    env->removeOutput( this, layer );
}

