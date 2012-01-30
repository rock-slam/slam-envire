#include "Core.hpp"
#include <boost/lexical_cast.hpp>

using namespace envire;

const std::string Operator::className = "envire::Operator";

Operator::Operator(int inputArity, int outputArity)
    : inputArity(inputArity), outputArity(outputArity)
{
}

Operator::Operator(Serialization& so, int inputArity, int outputArity)
    : EnvironmentItem(so)
    , inputArity(inputArity), outputArity(outputArity)
{
}

bool Operator::setInput( Layer* layer )
{
    removeInputs();
    return addInput(layer);
}

bool Operator::addInput( Layer* layer ) 
{
    if( inputArity && env->getInputs(this).size() >= static_cast<unsigned int>(inputArity) )
        throw std::runtime_error(className + " can only have " + boost::lexical_cast<std::string>(inputArity) + " inputs");
    env->addInput( this, layer );
    return true;
}

bool Operator::setOutput( Layer* layer )
{
    removeOutputs();
    return addOutput( layer );
}

bool Operator::addOutput( Layer* layer ) 
{
    if( outputArity && env->getOutputs(this).size() >= static_cast<unsigned int>(outputArity) )
        throw std::runtime_error(className + " can only have " + boost::lexical_cast<std::string>(outputArity) + " outputs");
    env->addOutput( this, layer );
    return true;
}

void Operator::removeInput( Layer* layer )
{
    env->removeInput( this, layer );
}

void Operator::removeInputs()
{
    env->removeInputs( this );
}

void Operator::removeOutput( Layer* layer )
{
    env->removeOutput( this, layer );
}

void Operator::removeOutputs()
{
    env->removeOutputs( this );
}

