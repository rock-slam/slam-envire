#include "Core.hpp"
#include "LaserScan.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

FrameNode::FrameNode( FrameNode_Ptr parent ) :
    parent(parent)
{
}

FrameNode::FrameNode()
{
}

bool FrameNode::isRoot() const
{
    return parent;
}

FrameNode_ConstPtr FrameNode::getParent() const
{
    return static_cast<FrameNode_ConstPtr>( static_cast<const FrameNode&>( *this ).getParent() );
}

FrameNode_Ptr FrameNode::getParent()
{
    if( isRoot() )
        throw std::runtime_error("Called getParent() on root FrameNode.");
    return parent;
}

Frame const& FrameNode::getTransform() const 
{
    return const_cast<Frame&>( static_cast<const FrameNode&>( *this ).getTransform() );
}

Frame& FrameNode::getTransform()
{
    if( isRoot() )
        throw std::runtime_error("Called getTransform() on root FrameNode.");
    return frame;
}

void FrameNode::setTransform(Frame const& transform)
{
    frame = transform;
}

Environment::Environment() 
{
    frame_tree = FrameNode_Ptr( new FrameNode() );
}

Frame Environment::relativeTransform(FrameNode const& from, FrameNode const& to)
{
    throw std::runtime_error("relativeTransform() Not implemented yet.");
}

void Environment::addLayer(Layer_Ptr layer)
{
    layers.push_back( layer );
}

void Environment::removeLayer(Layer_Ptr layer)
{
    layers.remove( layer );
}

FrameNode_Ptr Environment::getRootNode()
{
    return frame_tree;
}

bool Environment::loadSceneFile( const std::string& fileName, FrameNode_Ptr node )
{
    // TODO: define a better format for a scene file, which supports
    // hierarchical scenes
    std::ifstream ifile(fileName.c_str());
    if( ifile.fail() )
    {
        throw std::runtime_error("Could not open file '" + fileName + "'.");
    }
    else try
    {
        std::string line;
        while( !ifile.eof() ) {
            getline( ifile, line );
            std::istringstream iline( line );

            if( line.length() > 0 ) {
                std::string filePath;
                Eigen::Transform3f t;

                iline >> filePath;
                for(int j=0;j<16;j++) {
                    iline >> t.data()[j];
                }

                // TODO this is really a bit messy as it assumes that the
                // createFromScanFile creates a new FrameNode, which it might not.
                // since this code will change anyway, this has been left out so
                // far.
                LaserScan_Ptr scan = LaserScan::createFromScanFile( filePath, getRootNode() );
                Eigen::Quaternionf q = Eigen::Quaternionf(t.matrix().corner<3,3>(Eigen::TopLeft));
                Eigen::Vector3f v = t.translation();

                scan->getFrameNode()->getTransform().rotation = q;
                scan->getFrameNode()->getTransform().translation = v;
            }
        }

        ifile.close();
    } 
    catch( ... )
    {
        ifile.close();
        throw;
    }
}

bool Environment::loadSceneFile( const std::string& file ) 
{ 
    return loadSceneFile( file, getRootNode() );
}

bool Operator::addInput( Layer_Ptr layer ) 
{
    inputs.push_back( layer );
    return true;
}

bool Operator::addOutput( Layer_Ptr layer ) 
{
    layer->setGenerator( shared_from_this() );
    outputs.push_back( layer );
    return true;
}

void Operator::removeInput( Layer_Ptr layer )
{
    inputs.remove( layer );
}


void Operator::removeOutput( Layer_Ptr layer )
{
    outputs.remove( layer );
}

Layer::Layer(std::string const& id) : 
    id(id), immutable(false)
{
}

Layer::~Layer()
{
}

std::string Layer::getID() const
{
    return id;
}

bool Layer::isImmutable() const
{
    return immutable;
}

void Layer::setImmutable()
{
    immutable = true;
}

void Layer::resetDirty() 
{
    dirty = false;
}

void Layer::setDirty() 
{
    dirty = true;
}

bool Layer::isDirty() const
{
    return dirty;
}

bool Layer::detachFromOperator()
{
    if( isGenerated() ) 
        generator->removeOutput( shared_from_this() );

    generator = Operator_Ptr();
}

bool Layer::isGenerated() const 
{
    return generator;
}

bool Layer::setGenerator( Operator_Ptr generator ) 
{
   this->generator = generator; 
}

Operator_Ptr Layer::getGenerator() const
{
    return generator;
}

void Layer::updateFromOperator() 
{
    if( isGenerated() && isDirty() )
        generator->updateAll();

    dirty = false;
}

CartesianMap::CartesianMap(FrameNode_Ptr node, std::string const& id) :
    Layer(id), frame(node)
{
}

void CartesianMap::setFrameNode(FrameNode_Ptr frame)
{
    frame = frame;
}

FrameNode_Ptr CartesianMap::getFrameNode() 
{
    return frame;
}

FrameNode_ConstPtr CartesianMap::getFrameNode() const 
{
    return static_cast<FrameNode_ConstPtr>( static_cast<const CartesianMap&>( *this ).getFrameNode() );
}
