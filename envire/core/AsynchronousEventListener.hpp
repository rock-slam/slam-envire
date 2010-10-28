#ifndef __ENVIRE_ASYNCHRONOUSEVENTLISTENER__
#define __ENVIRE_ASYNCHRONOUSEVENTLISTENER__

#include <envire/Core.hpp>
#include <boost/thread/mutex.hpp>

namespace envire
{

struct EventMessage
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

    EventMessage( Type type, Operation operation, EnvironmentItem* a, EnvironmentItem* b = 0 );

    /** This method will check the message has any effect on the other message.
     * There are three possible scenarios which are returned by the result value.
     * 
     * IGNORE - no effect
     * INVALIDATE - the current message makes the other invalid
     * CANCEL - both messages cancel each other out 
     */ 
    Result effects( const EventMessage& other ) const;

    Type type;
    Operation operation;
    EnvironmentItem *a, *b;
};

class AsynchronousEventListener : public EventListener
{
    class Handler : public EventListener
    {
    public:
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

	void addEvent( const EventMessage& event );

	std::list<EventMessage> msgQueue;
	boost::mutex queueMutex;
    };

public:
    EventListener* getHandler();
    void processEvents();

protected:
    Handler handler;
};
}

#endif
