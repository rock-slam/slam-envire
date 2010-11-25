#include "Core.hpp"
#include "core/Event.hpp"

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

std::ostream& operator <<( std::ostream& ostream, Event::Type type )
{
    switch( type )
    {
	case Event::ITEM: ostream << "ITEM"; break;
	case Event::FRAMENODE_TREE: ostream << "FRAMENODE_TREE"; break;
	case Event::FRAMENODE: ostream << "FRAMENODE"; break;
	case Event::LAYER_TREE: ostream << "LAYER_TREE"; break;
	case Event::ROOT: ostream << "ROOT"; break;
    }
    return ostream;
}

std::ostream& operator <<( std::ostream& ostream, Event::Operation operation )
{
    switch( operation )
    {
	case Event::ADD: ostream << "ADD"; break;
	case Event::REMOVE: ostream << "REMOVE"; break;
	case Event::UPDATE: ostream << "UPDATE"; break;
    }
    return ostream;
}

std::ostream& operator <<( std::ostream& ostream, Event::Result result )
{
    switch( result )
    {
	case Event::IGNORE: ostream << "IGNORE"; break;
	case Event::INVALIDATE: ostream << "INVALIDATE"; break;
	case Event::CANCEL: ostream << "CANCEL"; break;
    }
    return ostream;
}

std::ostream& operator <<( std::ostream& ostream, const Event& msg )
{
    ostream << "(" << msg.operation << " " << msg.type << " a:" << msg.a << " b:" << msg.b << ")";
    return ostream;
}

Event::Event( Type type, Operation operation, EnvironmentItem* a, EnvironmentItem* b  )
    : type(type), operation(operation), a(a), b(b)
{
}

Event::Result Event::merge( const Event& other )
{
    if( operation == UPDATE )
    {
	return (type == other.type && operation == other.operation && a == other.a && b == other.b) ? INVALIDATE : IGNORE;
    }
    if( operation == REMOVE )
    {
	switch( type )
	{
	    case FRAMENODE_TREE: 
	    case FRAMENODE:
	    case LAYER_TREE:
		return (type == other.type && a == other.a && b == other.b) ? CANCEL : IGNORE;
	    case ITEM: 
		if( (a == other.a || a == other.b) && (other.operation == ADD || other.operation == UPDATE) )
		    return (type == other.type && other.operation == ADD) ? CANCEL : INVALIDATE;
		else
		    return IGNORE;
	    case ROOT:
		return (type == other.type && a == other.a) ? CANCEL : IGNORE;
	}
    }

    return IGNORE;
}

void EventListener::handle( const Event& event )
{
    if( event.type == Event::ITEM && event.operation == Event::ADD )
	itemAttached( event.a );
    else if( event.type == Event::ITEM && event.operation == Event::REMOVE )
	itemDetached( event.a );
    else if( event.type == Event::ITEM && event.operation == Event::UPDATE )
	itemModified( event.a );
    else if( event.type == Event::FRAMENODE_TREE && event.operation == Event::ADD )
	childAdded( static_cast<FrameNode*>(event.a), static_cast<FrameNode*>(event.b) );
    else if( event.type == Event::FRAMENODE_TREE && event.operation == Event::REMOVE )
	childRemoved( static_cast<FrameNode*>(event.a), static_cast<FrameNode*>(event.b) );
    else if( event.type == Event::LAYER_TREE && event.operation == Event::ADD )
	childAdded( static_cast<Layer*>(event.a), static_cast<Layer*>(event.b) );
    else if( event.type == Event::LAYER_TREE && event.operation == Event::REMOVE )
	childRemoved( static_cast<Layer*>(event.a), static_cast<Layer*>(event.b) );
    else if( event.type == Event::FRAMENODE && event.operation == Event::ADD )
	frameNodeSet( static_cast<CartesianMap*>(event.a), static_cast<FrameNode*>(event.b) );
    else if( event.type == Event::FRAMENODE && event.operation == Event::REMOVE )
	frameNodeDetached( static_cast<CartesianMap*>(event.a), static_cast<FrameNode*>(event.b) );
    else if( event.type == Event::ROOT && event.operation == Event::ADD )
	setRootNode( static_cast<FrameNode*>(event.a) );
    else if( event.type == Event::ROOT && event.operation == Event::REMOVE )
	removeRootNode( static_cast<FrameNode*>(event.a) );
    else 
	throw std::runtime_error("Event message not handled");
}

EventMessage::EventMessage( const Event& event )
    : Event( event )
{
    // store unique id's
    if( a ) id_a = a->getUniqueId();
    if( b ) id_b = b->getUniqueId();

    if( event.type == Event::ITEM && ( event.operation == ADD || event.operation == UPDATE ) )
    {
	// perform a copy of the EnvironmentItem in these cases
	a = a->clone();
    }
}

void EventMessage::apply( Environment* env )
{
}

