#ifndef __ENVIRE_EVENT__
#define __ENVIRE_EVENT__

#include <envire/Core.hpp>
#include <envire/core/EventSource.hpp>
#include <boost/thread/mutex.hpp>

namespace envire
{

/** Base Event class that encodes information on changes to a given environment.
 * Events can happen to different parts of the environment, the structure and/or
 * the data items. Each of these parts can have an add, remove or update event. 
 * Note, that not all combinations may always be legal (e.g. update is only
 * available for the individual items, not the structure). 
 */
struct Event
{
    enum Type 
    {
	ITEM,
	FRAMENODE_TREE,
	FRAMENODE,
	LAYER_TREE,
	ROOT
    };

    enum Operation
    {
	ADD,
	REMOVE,
	UPDATE
    };

    enum Result
    {
	IGNORE,
	INVALIDATE,
	CANCEL
    };

    /** Constructor for an event, given the following parameters
     *
     * @param type - the type of data that is changed
     * @param operation - what is happening on the data
     * @param a - the first subject of the change
     * @param b - optional second subject of the change
     */
    Event( Type type, Operation operation, EnvironmentItem* a, EnvironmentItem* b = 0 );

    /** This method will check the message has any effect on the other message.
     * There are three possible scenarios which are returned by the result value.
     * 
     * IGNORE - no effect
     * INVALIDATE - the current message makes the other invalid
     * CANCEL - both messages cancel each other out 
     */ 
    virtual Result merge( const Event& other );

    Type type;
    Operation operation;
    EnvironmentItem *a, *b;
};

/** The EventMessage class is derived from Event, and holds information about an
 * environment in a thread safe manner. It does that by copying relevant
 * information about the environment, when passed an event in the constructor.
 * EventMessages in a way encode information about an environment, but are not
 * bound to a particular instance. They can be used to copy environments between
 * threads or processes.
 */
struct EventMessage : public Event
{
    long id_a;
    long id_b;

    EventMessage( const Event& event );
    virtual void apply( Environment* env );
};

/** An EventHandler can be registered with an environment, and is then called,
 * whenever an new Event is generated within the Environment
 */
class EventHandler
{
public:
    virtual void handle( const Event& message ) = 0;
};

/** Event dispatcher interface class. Override the callbacks methods that you
 * are interested in.
 */
class EventDispatcher
{
public:
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

/** EventHandler that will apply the handled events to the given Environment
 */
class EventProcessor : public EventListener
{
    Environment *env;

public:
    EventProcessor( Environment *env );

    void itemAttached(EnvironmentItem *item);
    void itemDetached(EnvironmentItem *item);
    void childAdded(FrameNode* parent, FrameNode* child);
    void childAdded(Layer* parent, Layer* child);

    void frameNodeSet(CartesianMap* map, FrameNode* node);
    void frameNodeDetached(CartesianMap* map, FrameNode* node);

    void childRemoved(FrameNode* parent, FrameNode* child);
    void childRemoved(Layer* parent, Layer* child);

    void setRootNode(FrameNode *root);
    void removeRootNode(FrameNode *root);

    void itemModified(EnvironmentItem *item);
};

class EventQueue : public EventHandler, public EventSource
{
    std::list<EventMessage> msgQueue;
    boost::mutex queueMutex;

public:
    void handle( const Event& message );
    void flush();
};
    
}

#endif
