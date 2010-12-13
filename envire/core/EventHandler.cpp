#include "core/EventHandler.hpp"

using namespace envire;

void EventDispatcher::itemAttached(EnvironmentItem *item) {}
void EventDispatcher::itemDetached(EnvironmentItem *item) {}
void EventDispatcher::childAdded(FrameNode* parent, FrameNode* child) {}
void EventDispatcher::childAdded(Layer* parent, Layer* child) {}
void EventDispatcher::frameNodeSet(CartesianMap* map, FrameNode* node) {}
void EventDispatcher::frameNodeDetached(CartesianMap* map, FrameNode* node) {}
void EventDispatcher::childRemoved(FrameNode* parent, FrameNode* child) {}
void EventDispatcher::childRemoved(Layer* parent, Layer* child) {}
void EventDispatcher::setRootNode(FrameNode *root) {}
void EventDispatcher::removeRootNode(FrameNode *root) {}
void EventDispatcher::itemModified(EnvironmentItem *item) {}

void EventListener::handle( const Event& event )
{
    dispatch( event.type, event.operation, event.a.get(), event.b.get(), this );
}

void EventDispatcher::dispatch( Event::Type type, Event::Operation operation, EnvironmentItem* a, EnvironmentItem* b, EventDispatcher* disp )
{
    if( type == Event::ITEM && operation == Event::ADD )
	disp->itemAttached( a );
    else if( type == Event::ITEM && operation == Event::REMOVE )
	disp->itemDetached( a );
    else if( type == Event::ITEM && operation == Event::UPDATE )
	disp->itemModified( a );
    else if( type == Event::FRAMENODE_TREE && operation == Event::ADD )
	disp->childAdded( static_cast<FrameNode*>(a), static_cast<FrameNode*>(b) );
    else if( type == Event::FRAMENODE_TREE && operation == Event::REMOVE )
	disp->childRemoved( static_cast<FrameNode*>(a), static_cast<FrameNode*>(b) );
    else if( type == Event::LAYER_TREE && operation == Event::ADD )
	disp->childAdded( static_cast<Layer*>(a), static_cast<Layer*>(b) );
    else if( type == Event::LAYER_TREE && operation == Event::REMOVE )
	disp->childRemoved( static_cast<Layer*>(a), static_cast<Layer*>(b) );
    else if( type == Event::FRAMENODE && operation == Event::ADD )
	disp->frameNodeSet( static_cast<CartesianMap*>(a), static_cast<FrameNode*>(b) );
    else if( type == Event::FRAMENODE && operation == Event::REMOVE )
	disp->frameNodeDetached( static_cast<CartesianMap*>(a), static_cast<FrameNode*>(b) );
    else if( type == Event::ROOT && operation == Event::ADD )
	disp->setRootNode( static_cast<FrameNode*>(a) );
    else if( type == Event::ROOT && operation == Event::REMOVE )
	disp->removeRootNode( static_cast<FrameNode*>(a) );
    else 
	throw std::runtime_error("Event message not handled");
}

