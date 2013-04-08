#include "EventHandler.hpp"
#include "FrameNode.hpp"
#include "Layer.hpp"
#include <envire/Core.hpp>

using namespace envire;

void EventFilter::handle( const Event& message )
{
    handler->handle( message );
}

void EventHandler::receive( const Event& message )
{
    // if there is a filter apply and return if
    // we are not allowed to pass the message
    if( filter && !filter->filter( message ) )
	return;

    handle( message );
}

void EventHandler::setFilter( EventFilter* filter ) 
{ 
    this->filter = filter; 
    filter->setHandler( this );
}

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

void EventDispatcher::dispatch( event::Type type, event::Operation operation, EnvironmentItem* a, EnvironmentItem* b, EventDispatcher* disp )
{
    if( type == event::ITEM && operation == event::ADD )
	disp->itemAttached( a );
    else if( type == event::ITEM && operation == event::REMOVE )
	disp->itemDetached( a );
    else if( type == event::ITEM && operation == event::UPDATE )
	disp->itemModified( a );
    else if( type == event::FRAMENODE_TREE && operation == event::ADD )
	disp->childAdded( static_cast<FrameNode*>(a), static_cast<FrameNode*>(b) );
    else if( type == event::FRAMENODE_TREE && operation == event::REMOVE )
	disp->childRemoved( static_cast<FrameNode*>(a), static_cast<FrameNode*>(b) );
    else if( type == event::LAYER_TREE && operation == event::ADD )
	disp->childAdded( static_cast<Layer*>(a), static_cast<Layer*>(b) );
    else if( type == event::LAYER_TREE && operation == event::REMOVE )
	disp->childRemoved( static_cast<Layer*>(a), static_cast<Layer*>(b) );
    else if( type == event::FRAMENODE && operation == event::ADD )
	disp->frameNodeSet( static_cast<CartesianMap*>(a), static_cast<FrameNode*>(b) );
    else if( type == event::FRAMENODE && operation == event::REMOVE )
	disp->frameNodeDetached( static_cast<CartesianMap*>(a), static_cast<FrameNode*>(b) );
    else if( type == event::ROOT && operation == event::ADD )
	disp->setRootNode( static_cast<FrameNode*>(a) );
    else if( type == event::ROOT && operation == event::REMOVE )
	disp->removeRootNode( static_cast<FrameNode*>(a) );
    else 
	throw std::runtime_error("Event message not handled");
}

void EventQueue::handle( const Event& message )
{
    boost::lock_guard<boost::mutex> lock( queueMutex );
    std::list<Event>::iterator it = msgQueue.begin();
    // create a local copy of the event
    Event event(message);
    event.ref( m_async );

    bool valid = true;
    while( it != msgQueue.end() )
    {
	event::Result res = 
	    event.merge( *it );

	// either cancel or invalidate make the old event
	// obsolete
	if( res == event::CANCEL || res == event::INVALIDATE )
        {
	    it = msgQueue.erase( it );
        }
        else
            ++it;

	// cancel will also make the current event obsolete
	if( res == event::CANCEL )
	    valid = false;
    }

    if( valid )
    {
	msgQueue.push_back( event );
    }
}

void EventQueue::flush()
{
    boost::lock_guard<boost::mutex> lock( queueMutex );
    for( std::list<Event>::iterator it = msgQueue.begin(); it != msgQueue.end(); it++ )
    {
	process( *it );
    }
    msgQueue.clear();
}

EventProcessor::EventProcessor( Environment *env )
    : env( env ) {}

void EventProcessor::process( const Event& message )
{
    message.apply( env );
}

