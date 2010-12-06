#ifndef __ENVIRE_EVENTSOURCE__
#define __ENVIRE_EVENTSOURCE__

#include <vector>

namespace envire
{
class Event;
class EventHandler;

/** Base for any class that wants to provide Events
 */
class EventSource 
{
    std::vector<EventHandler *> eventHandlers;

public:
    /**
     * Adds an eventHandler that gets called whenever there
     * are modifications to the evironment.
     * Note, the handler gets called from the evire thread context.
     */
    void addEventHandler(EventHandler *handler);

    /**
     * Remove the given eventHandler fromt the environment
     */
    void removeEventHandler(EventHandler *handler);

    /**
     * will pass the @param event to the registered event handlers
     */
    void emit( const Event& event );
};

}

#endif
