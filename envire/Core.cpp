#include "Core.hpp"
#include "LaserScan.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <stdexcept>
#include <fstream>
#include <string>

using namespace envire;

FrameNode::FrameNode()
{
}

bool FrameNode::isRoot() const
{
    return !parent;
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

void FrameNode::setParent(FrameNode_Ptr node) 
{
    if( parent ) 
        parent->frameNodes.remove( shared_from_this() );

    if( node ) 
        node->frameNodes.push_back( shared_from_this() );

    parent = node;
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

std::list<FrameNode_Ptr>::const_iterator FrameNode::beginNodes()
{
    return frameNodes.begin();
}

std::list<FrameNode_Ptr>::const_iterator FrameNode::endNodes()
{
    return frameNodes.end();
}

std::list<CartesianMap_Ptr>::const_iterator FrameNode::beginMaps()
{
    return maps.begin();
}

std::list<CartesianMap_Ptr>::const_iterator FrameNode::endMaps()
{
    return maps.end();
}

Environment::Environment(FrameNode_Ptr node, std::string const& id) : 
    CartesianMap(node, id)
{
}

Environment::Environment(const std::string& id) :
    CartesianMap(id)
{
    // provide a FrameNode if there is none provided
    frame = FrameNode_Ptr( new FrameNode() );
}

Frame Environment::relativeTransform(FrameNode const& from, FrameNode const& to)
{
    throw std::runtime_error("relativeTransform() Not implemented yet.");
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
                LaserScan_Ptr scan = LaserScan::createFromScanFile( filePath, getFrameNode() );
                Eigen::Quaternionf q = Eigen::Quaternionf(t.matrix().corner<3,3>(Eigen::TopLeft));
                Eigen::Vector3f v = t.translation();

                if( scan->getFrameNode()->isRoot() )
                {
                    FrameNode_Ptr node = FrameNode_Ptr( new envire::FrameNode() );
                    node->setParent( scan->getFrameNode() );
                    scan->setFrameNode( node );
                }

                scan->getFrameNode()->getTransform().getRotation() = q;
                scan->getFrameNode()->getTransform().getTranslation() = v;
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
    return loadSceneFile( file, getFrameNode() );
}

Layer_Ptr Environment::clone(const std::string& id)
{
    Environment *c = new Environment(*this);
    c->id = id;
    Layer_Ptr clone = Layer_Ptr(c);
    return clone;
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
    id(id), immutable(false), generator(Operator_Ptr())
{
}

Layer::~Layer()
{
}

void Layer::setParent( Layer_Ptr parent ) 
{
    if( this->parent )
        this->parent->removeLayer( shared_from_this() );
    
    if( parent )
        parent->addLayer( shared_from_this() );

    this->parent = parent;
}

Layer_Ptr Layer::getParent()
{
    return parent;
}

void Layer::addLayer( Layer_Ptr layer )
{
    if( layer->parent )
        layer->parent->removeLayer( shared_from_this() );
    
    layers.push_back( layer );
    layer->parent = shared_from_this();
}

void Layer::removeLayer( Layer_Ptr layer )
{
    layers.remove( layer );
    layer->parent = Layer_Ptr();
}

std::list<Layer_Ptr>::const_iterator Layer::beginLayers()
{
    return layers.begin();
}

std::list<Layer_Ptr>::const_iterator Layer::endLayers()
{
    return layers.end();
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

CartesianMap::CartesianMap(std::string const& id) :
    Layer(id)
{
}

void CartesianMap::setFrameNode(FrameNode_Ptr frame)
{
    if( this->frame )
        this->frame->maps.remove( boost::static_pointer_cast<envire::CartesianMap>(shared_from_this()) );

    this->frame = frame;
    frame->maps.push_back( boost::static_pointer_cast<envire::CartesianMap>(shared_from_this()) );
}

FrameNode_Ptr CartesianMap::getFrameNode() 
{
    return frame;
}

FrameNode_ConstPtr CartesianMap::getFrameNode() const 
{
    return static_cast<FrameNode_ConstPtr>( static_cast<const CartesianMap&>( *this ).getFrameNode() );
}
