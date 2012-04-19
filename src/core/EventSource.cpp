#include "EventSource.hpp"
#include "EventHandler.hpp"
#include "Event.hpp"

#include <boost/lambda/lambda.hpp>

using namespace envire;

void EventSource::handle( const Event& event )
{
    // pass the event on to the handlers
    for(std::vector<EventHandler*>::iterator it = eventHandlers.begin(); it != eventHandlers.end(); it++ )
    {
	(*it)->receive( event );
    }
}

void EventSource::addEventHandler(EventHandler *handler) 
{
    eventHandlers.push_back(handler);
}

void EventSource::removeEventHandler(EventHandler *handler) 
{
    eventHandlers.erase( 
	    std::remove_if( eventHandlers.begin(), eventHandlers.end(), boost::lambda::_1 == handler ), 
	    eventHandlers.end() );
}
