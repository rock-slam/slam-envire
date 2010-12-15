#include "EventSource.hpp"
#include "EventHandler.hpp"
#include "Event.hpp"

using namespace envire;

void EventSource::handle( const Event& event )
{
    // pass the event on to the handlers
    for(std::vector<EventHandler*>::iterator it = eventHandlers.begin(); it != eventHandlers.end(); it++ )
    {
	(*it)->handle( event );
    }
}

void EventSource::addEventHandler(EventHandler *handler) 
{
    eventHandlers.push_back(handler);
}

void EventSource::removeEventHandler(EventHandler *handler) 
{
    std::vector<EventHandler*>::iterator it = std::find(eventHandlers.begin(), eventHandlers.end(), handler);
    if(it != eventHandlers.end()) {
	eventHandlers.erase(it);
    }
}
