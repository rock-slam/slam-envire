#ifndef __ENVIRE_EVENTHANDLER__
#define __ENVIRE_EVENTHANDLER__

#include <list>

#include <envire/core/Event.hpp>
#include <envire/core/EventSource.hpp>
#include <boost/thread/mutex.hpp>

namespace envire
{

class EventHandler;
class EnvironmentItem;
class FrameNode;
class Layer;
class CartesianMap;
class Environment;

class EventFilter
{
public:
    EventFilter() : handler(NULL) {}

    /** @brief callback for eventfilter
     * Should return true if the message can be passed
     */
    virtual bool filter( envire::Event const& ) = 0;

    /** @brief this function will get called 
     * by the handler, once the filter is attached.
     */
    void setHandler( EventHandler* handler ) { this->handler = handler; }

protected:
    /** @brief pass a message to the handler directly
     */
    void handle( const Event& message );

    EventHandler *handler;
};

/** An EventHandler can be registered with an environment, and is then called,
 * whenever an new Event is generated within the Environment
 */
class EventHandler
{
    friend class EventFilter;

public:
    EventHandler() : filter(NULL) {}

    /** @brief call this message to pass a message to the EventHandler
     */
    void receive( const Event& message );

    /** @brief set optional event filter
     */
    void setFilter( EventFilter* filter );

protected:
    /** @brief callback method for possibly filtered events
     */
    virtual void handle( const Event& message ) = 0;
    
    EventFilter* filter;
};

/** Event dispatcher interface class. Override the callbacks methods that you
 * are interested in.
 */
class EventDispatcher
{
public:
    static void dispatch( event::Type type, event::Operation operation, EnvironmentItem* a, EnvironmentItem* b, EventDispatcher* dispatch );

    virtual void itemAttached(EnvironmentItem *item);
    virtual void itemDetached(EnvironmentItem *item);
    virtual void childAdded(FrameNode* parent, FrameNode* child);
    virtual void childAdded(Layer* parent, Layer* child);

    virtual void frameNodeSet(CartesianMap* map, FrameNode* node);
    virtual void frameNodeDetached(CartesianMap* map, FrameNode* node);

    virtual void childRemoved(FrameNode* parent, FrameNode* child);
    virtual void childRemoved(Layer* parent, Layer* child);

    virtual void setRootNode(FrameNode *root);
    virtual void removeRootNode(FrameNode *root);

    virtual void itemModified(EnvironmentItem *item);
};

/** Convenience class, that performs the dispatching of particular event
 * types into callback methods.
 */
class EventListener : public EventHandler, public EventDispatcher
{
public:
    void handle( const Event& message );
};

class EventQueue : public EventHandler
{
protected:
    std::list<Event> msgQueue;
    boost::mutex queueMutex;
    bool m_async;

    /** Callback that is called by the environment as an event handler. You
     * should normally not reimplement this
     */
    void handle( const Event& message );

public:
    EventQueue() : m_async( false ) {}

    /** @brief if set to true, the eventqueue will allow the
     * messages to be processed in a different thread.
     *
     * Note: this does have a performance impact, since the messages
     * need to create copies of the environment items, instead of 
     * just referencing them. 
     */
    void allowMultithreading( bool allow ) { m_async = allow; }
	
    /** Send all events currently stored in the queue to process
     */
    void flush();

    /** Callback called with each queued event when flush() is called
     */
    virtual void process( const Event& message ) = 0;
};

/** EventHandler that will apply the handled events to the given Environment
 */
class EventProcessor : public EventQueue
{
    Environment *env;

public:
    EventProcessor( Environment *env );
    void process( const Event& message );
};

}

#endif
