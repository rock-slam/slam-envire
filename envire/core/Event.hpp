#ifndef __ENVIRE_EVENT__
#define __ENVIRE_EVENT__

#include <envire/Core.hpp>
#include <boost/shared_ptr.hpp>

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
    Event( Type type, Operation operation, EnvironmentItem::Ptr a, EnvironmentItem::Ptr b = 0 );

    /** This method will check the message has any effect on the other message.
     * There are three possible scenarios which are returned by the result value.
     * 
     * IGNORE - no effect
     * INVALIDATE - the current message makes the other invalid
     * CANCEL - both messages cancel each other out 
     */ 
    Result merge( const Event& other );

    /** will apply the changes this event represents to the give environment.
     * Event needs to have the ref function called before it can be applied.
     */
    void apply( Environment* env ) const;

    /** resolve the references to the items referenced in the event, and
     * perform copies if needed.
     */
    void ref();

    Type type;
    Operation operation;
    EnvironmentItem::Ptr a, b;
    long id_a, id_b;
};

}

#endif
