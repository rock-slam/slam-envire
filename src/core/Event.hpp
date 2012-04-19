#ifndef __ENVIRE_EVENT__
#define __ENVIRE_EVENT__

#include <envire/Core.hpp>
#include <boost/shared_ptr.hpp>
#include <envire/core/EventTypes.hpp>

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
    /** Constructor for an event, given the following parameters
     *
     * @param type - the type of data that is changed
     * @param operation - what is happening on the data
     * @param a - the first subject of the change
     * @param b - optional second subject of the change
     */
    Event( event::Type type, event::Operation operation, EnvironmentItem::Ptr a, EnvironmentItem::Ptr b = 0 );

    /** This method will check the message has any effect on the other message.
     * There are three possible scenarios which are returned by the result value.
     * 
     * IGNORE - no effect
     * INVALIDATE - the current message makes the other invalid
     * CANCEL - both messages cancel each other out 
     */ 
    event::Result merge( const Event& other );

    /** will apply the changes this event represents to the give environment.
     * Event needs to have the ref function called before it can be applied.
     */
    void apply( Environment* env ) const;

    /** resolve the references to the items referenced in the event, and
     * perform copies if needed. 
     *
     * @param set to false if copying should be omitted.
     */
    void ref( bool clone );

    event::Type type;
    event::Operation operation;
    EnvironmentItem::Ptr a, b;
    std::string id_a, id_b;
};

std::ostream& operator <<( std::ostream& ostream, event::Type type );
std::ostream& operator <<( std::ostream& ostream, event::Operation operation );
std::ostream& operator <<( std::ostream& ostream, event::Result result );
std::ostream& operator <<( std::ostream& ostream, const Event& msg );

}

#endif
