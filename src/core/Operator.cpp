#include "Core.hpp"
#include "Operator.hpp"
#include <boost/lexical_cast.hpp>

using namespace envire;

const std::string Operator::className = "envire::Operator";

Operator::Operator(std::string const& id, int inputArity, int outputArity)
    : EnvironmentItem(id), inputArity(inputArity), outputArity(outputArity)
{
}

Operator::Operator(int inputArity, int outputArity)
    : EnvironmentItem(Environment::ITEM_NOT_ATTACHED)
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
    if (isAttached() == false)
        throw std::runtime_error("Before adding the input attach " + className + " to the environment.");

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
    if (isAttached() == false)
        throw std::runtime_error("Before adding the output attach " + className + " to the environment.");

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

