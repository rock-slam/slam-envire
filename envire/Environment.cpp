#include "Core.hpp"

#include <algorithm>
#include <utility>
#include <stdexcept>

using namespace std;
using namespace envire;

long EnvironmentItem::last_id = 0;

EnvironmentItem::EnvironmentItem()
    : env(NULL), unique_id(last_id++)
{
}

EnvironmentItem::EnvironmentItem(Environment* envPtr)
    : env(envPtr), unique_id(last_id++)
{
}

EnvironmentItem::~EnvironmentItem()
{
}

bool EnvironmentItem::isAttached() const
{
    return env;
}

long EnvironmentItem::getUniqueId() const
{
    return unique_id;
}

Environment* EnvironmentItem::getEnvironment() 
{
    return env;
}



Environment::Environment()
{
    // each environment has a root node
    rootNode = new FrameNode();
    // also put it in the same managed process
    attachItem( rootNode );
}

Environment::~Environment() 
{
    // perform a delete on all the owned objects
    for(itemListType::iterator it=items.begin();it!=items.end();++it)
    {
	delete *it;
    }
}

void Environment::attachItem(EnvironmentItem* item)
{
    // first make sure item not already present
    if( find( items.begin(), items.end(), item ) != items.end() )
	return;

    // add item to internal list
    items.push_back(item);

    // set a pointer to environment object
    item->env = this;
} 

void Environment::detachItem(EnvironmentItem* item)
{
    items.remove(item);
}

void Environment::addChild(FrameNode* parent, FrameNode* child)
{
    frameNodeTree.insert(make_pair(child, parent));
}

void Environment::addChild(Layer* parent, Layer* child)
{
    layerTree.insert(make_pair(child, parent));
}

void Environment::removeChild(FrameNode* parent, FrameNode* child)
{
    frameNodeTree.erase( frameNodeTree.find( child ) );
}

void Environment::removeChild(Layer* parent, Layer* child) 
{
    layerTree.erase( layerTree.find( child ) );
}

FrameNode* Environment::getParent(FrameNode* node) 
{
    frameNodeTreeType::iterator it = frameNodeTree.find( node );
    if( it == frameNodeTree.end() )
	return NULL;
    else
	return it->second;
}

Layer* Environment::getParent(Layer* layer) 
{
    layerTreeType::iterator it = layerTree.find( layer );
    if( it == layerTree.end() )
	return NULL;
    else
	return it->second;
}

FrameNode* Environment::getRootNode() 
{
    return rootNode;
}

std::list<FrameNode*> Environment::getChildren(FrameNode* parent)
{
    std::list<FrameNode*> children;
    for(frameNodeTreeType::iterator it=frameNodeTree.begin();it != frameNodeTree.end(); ++it )
    {
	if( it->second == parent )
	    children.push_back( it->first );
    }

    return children;
}

std::list<Layer*> Environment::getChildren(Layer* parent) 
{
    std::list<Layer*> children;
    for(layerTreeType::iterator it=layerTree.begin();it != layerTree.end(); ++it )
    {
	if( it->second == parent )
	    children.push_back( it->first );
    }

    return children;
}

void Environment::setFrameNode(CartesianMap* map, FrameNode* node)
{
    cartesianMapGraph.insert( make_pair( map, node ) );
}

FrameNode* Environment::getFrameNode(CartesianMap* map)
{
    return cartesianMapGraph[map];
}

std::list<CartesianMap*> Environment::getMaps(FrameNode* node) 
{
    std::list<CartesianMap*> maps;
    for(cartesianMapGraphType::iterator it=cartesianMapGraph.begin();it != cartesianMapGraph.end(); ++it )
    {
	if( it->second == node )
	    maps.push_back( it->first );
    }
    return maps;
}

bool Environment::addInput(Operator* op, Layer* input)
{
    operatorGraphInput.insert( make_pair(op, input) );
}

bool Environment::addOutput(Operator* op, Layer* output)
{
    operatorGraphOutput.insert( make_pair(op, output) );
}

bool Environment::removeInput(Operator* op, Layer* input)
{
    for(operatorGraphType::iterator it=operatorGraphInput.begin();it != operatorGraphInput.end();)
    {
	if( it->first == op && it->second == input )
	    operatorGraphInput.erase( it++ );
	else
	    ++it;
    }
}

bool Environment::removeOutput(Operator* op, Layer* output)
{
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();)
    {
	if( it->first == op && it->second == output )
	    operatorGraphOutput.erase( it++ );
	else
	    ++it;
    }
}

std::list<Layer*> Environment::getInputs(Operator* op)
{
    std::list<Layer*> inputs;
    for(operatorGraphType::iterator it=operatorGraphInput.begin();it != operatorGraphInput.end();++it)
    {
	if( it->first == op )
	    inputs.push_back( it->second );
    }
    return inputs;
}

std::list<Layer*> Environment::getOutputs(Operator* op) 
{
    std::list<Layer*> outputs;
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();++it)
    {
	if( it->first == op )
	    outputs.push_back( it->second );
    }
    return outputs;
}

Operator* Environment::getGenerator(Layer* output) 
{
    for(operatorGraphType::iterator it=operatorGraphOutput.begin();it != operatorGraphOutput.end();++it)
    {
	if( it->second == output )
	    return it->first;
    }
}


Frame Environment::relativeTransform(const FrameNode* from, const FrameNode* to)
{
    throw std::runtime_error("relativeTransform() Not implemented yet.");
}

bool Environment::loadSceneFile( const std::string& fileName )
{
    // TODO: define a better format for a scene file, which supports
    // hierarchical scenes
#if 0
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
#endif
}

