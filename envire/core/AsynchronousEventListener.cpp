#include "AsynchronousEventListener.hpp"

using namespace envire;

EventMessage::Result EventMessage::effects( const EventMessage& other ) const
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
		    return (type == other.type) ? CANCEL : INVALIDATE;
		else
		    return IGNORE;
	    case ROOT:
		return (type == other.type && a == other.a) ? CANCEL : IGNORE;
	}
    }

    return IGNORE;
}

EventMessage::EventMessage( Type type, Operation operation, EnvironmentItem* a, EnvironmentItem* b  )
    : type(type), operation(operation), a(a), b(b)
{
}

void AsynchronousEventListener::Handler::addEvent( const EventMessage& event )
{
    boost::mutex::scoped_lock lock( queueMutex );
    std::list<EventMessage>::iterator it = msgQueue.begin();
    bool valid = true;
    while( it != msgQueue.end() )
    {
	EventMessage::Result res = 
	    event.effects( *it );

	if( res == EventMessage::CANCEL || res == EventMessage::INVALIDATE )
	    it = msgQueue.erase( it );

	if( res == EventMessage::CANCEL )
	    valid = false;

	++it;
    }

    if( valid )
	msgQueue.push_back( event );
}

void AsynchronousEventListener::Handler::itemAttached(EnvironmentItem *item)
{
    addEvent( EventMessage( EventMessage::ITEM, EventMessage::ADD, item ) );
}

void AsynchronousEventListener::Handler::itemDetached(EnvironmentItem *item)
{
    addEvent( EventMessage( EventMessage::ITEM, EventMessage::REMOVE, item ) );
}

void AsynchronousEventListener::Handler::childAdded(FrameNode* parent, FrameNode* child)
{
    addEvent( EventMessage( EventMessage::FRAMENODE_TREE, EventMessage::ADD, parent, child ) );
}

void AsynchronousEventListener::Handler::childAdded(Layer* parent, Layer* child)
{
    addEvent( EventMessage( EventMessage::LAYER_TREE, EventMessage::ADD, parent, child ) );
}

void AsynchronousEventListener::Handler::frameNodeSet(CartesianMap* map, FrameNode* node)
{
    addEvent( EventMessage( EventMessage::FRAMENODE, EventMessage::ADD, map, node ) );
}

void AsynchronousEventListener::Handler::frameNodeDetached(CartesianMap* map, FrameNode* node)
{
    addEvent( EventMessage( EventMessage::FRAMENODE, EventMessage::REMOVE, map, node ) );
}

void AsynchronousEventListener::Handler::childRemoved(FrameNode* parent, FrameNode* child)
{
    addEvent( EventMessage( EventMessage::FRAMENODE_TREE, EventMessage::REMOVE, parent, child ) );
}

void AsynchronousEventListener::Handler::childRemoved(Layer* parent, Layer* child)
{
    addEvent( EventMessage( EventMessage::LAYER_TREE, EventMessage::REMOVE, parent, child ) );
}

void AsynchronousEventListener::Handler::setRootNode(FrameNode *root)
{
    addEvent( EventMessage( EventMessage::ROOT, EventMessage::ADD, root ) );
}

void AsynchronousEventListener::Handler::removeRootNode(FrameNode *root)
{
    addEvent( EventMessage( EventMessage::ROOT, EventMessage::REMOVE, root ) );
}

void AsynchronousEventListener::Handler::itemModified(EnvironmentItem *item)
{
    addEvent( EventMessage( EventMessage::ITEM, EventMessage::UPDATE, item ) );
}

EventListener* AsynchronousEventListener::getHandler()
{
    return &handler;
}

bool AsynchronousEventListener::hasEvents() const
{
    return !handler.msgQueue.empty();
}

void AsynchronousEventListener::processEvents()
{
    boost::mutex::scoped_lock lock( handler.queueMutex );
    for( std::list<EventMessage>::iterator it = handler.msgQueue.begin(); it != handler.msgQueue.end(); it++ )
    {
	EventMessage &event( *it );
	if( event.type == EventMessage::ITEM && event.operation == EventMessage::ADD )
	    itemAttached( event.a );
	else if( event.type == EventMessage::ITEM && event.operation == EventMessage::REMOVE )
	    itemDetached( event.a );
	else if( event.type == EventMessage::ITEM && event.operation == EventMessage::UPDATE )
	    itemModified( event.a );
	else if( event.type == EventMessage::FRAMENODE_TREE && event.operation == EventMessage::ADD )
	    childAdded( static_cast<FrameNode*>(event.a), static_cast<FrameNode*>(event.b) );
	else if( event.type == EventMessage::FRAMENODE_TREE && event.operation == EventMessage::REMOVE )
	    childRemoved( static_cast<FrameNode*>(event.a), static_cast<FrameNode*>(event.b) );
	else if( event.type == EventMessage::LAYER_TREE && event.operation == EventMessage::ADD )
	    childAdded( static_cast<Layer*>(event.a), static_cast<Layer*>(event.b) );
	else if( event.type == EventMessage::LAYER_TREE && event.operation == EventMessage::REMOVE )
	    childRemoved( static_cast<Layer*>(event.a), static_cast<Layer*>(event.b) );
	else if( event.type == EventMessage::FRAMENODE && event.operation == EventMessage::ADD )
	    frameNodeSet( static_cast<CartesianMap*>(event.a), static_cast<FrameNode*>(event.b) );
	else if( event.type == EventMessage::FRAMENODE && event.operation == EventMessage::REMOVE )
	    frameNodeDetached( static_cast<CartesianMap*>(event.a), static_cast<FrameNode*>(event.b) );
	else if( event.type == EventMessage::ROOT && event.operation == EventMessage::ADD )
	    setRootNode( static_cast<FrameNode*>(event.a) );
	else if( event.type == EventMessage::ROOT && event.operation == EventMessage::REMOVE )
	    removeRootNode( static_cast<FrameNode*>(event.a) );
	else 
	    throw std::runtime_error("Event message not handled");
    }
    handler.msgQueue.clear();
}
