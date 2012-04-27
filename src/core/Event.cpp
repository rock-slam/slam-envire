#include "Core.hpp"
#include "core/Event.hpp"
#include "core/EventHandler.hpp"

using namespace envire;

std::ostream& envire::operator <<( std::ostream& ostream, event::Type type )
{
    switch( type )
    {
	case event::ITEM: ostream << "ITEM"; break;
	case event::FRAMENODE_TREE: ostream << "FRAMENODE_TREE"; break;
	case event::FRAMENODE: ostream << "FRAMENODE"; break;
	case event::LAYER_TREE: ostream << "LAYER_TREE"; break;
	case event::ROOT: ostream << "ROOT"; break;
    }
    return ostream;
}

std::ostream& envire::operator <<( std::ostream& ostream, event::Operation operation )
{
    switch( operation )
    {
	case event::ADD: ostream << "ADD"; break;
	case event::REMOVE: ostream << "REMOVE"; break;
	case event::UPDATE: ostream << "UPDATE"; break;
    }
    return ostream;
}

std::ostream& envire::operator <<( std::ostream& ostream, event::Result result )
{
    switch( result )
    {
	case event::IGNORE: ostream << "IGNORE"; break;
	case event::INVALIDATE: ostream << "INVALIDATE"; break;
	case event::CANCEL: ostream << "CANCEL"; break;
    }
    return ostream;
}

std::ostream& envire::operator <<( std::ostream& ostream, const Event& msg )
{
    ostream << "(" 
	<< msg.operation 
	<< " " << msg.type 
	<< " a:" << msg.a 
	<< " b:" << msg.b 
	<< " id_a:" << msg.id_a 
	<< " id_b:" << msg.id_b 
	<< ")";
    return ostream;
}

Event::Event( event::Type type, event::Operation operation, EnvironmentItem::Ptr a, EnvironmentItem::Ptr b  )
    : type(type), operation(operation), a(a), b(b), id_a( Environment::ITEM_NOT_ATTACHED ), id_b( Environment::ITEM_NOT_ATTACHED )
{
}

event::Result Event::merge( const Event& other )
{
    // overwrite event if its the same 
    if(type == other.type && operation == other.operation && id_a == other.id_a && id_b == other.id_b) 
	return event::INVALIDATE;

    // adding and removing will cancel each other, if the events are otherwise the same
    if( ((operation == event::REMOVE && other.operation == event::ADD)
        || (operation == event::ADD && other.operation == event::REMOVE))
	&& type == other.type && id_a == other.id_a && id_b == other.id_b )
	return event::CANCEL;

    // removing an item will also invalidate update events
    if( (operation == event::REMOVE && other.operation == event::UPDATE)
	&& type == other.type && id_a == other.id_a && id_b == other.id_b )
	return event::INVALIDATE;

    return event::IGNORE;
}

void Event::ref( bool clone )
{
    // store unique id's
    if( a ) id_a = a->getUniqueId();
    if( b ) id_b = b->getUniqueId();

    if( type == event::ITEM && ( operation == event::ADD || operation == event::UPDATE ) )
    {
	// perform a copy of the EnvironmentItem in these cases
	// and already set the unique_id to the source id
	if( clone )
	    a = a->clone();

	a->unique_id = id_a;
	b = 0;
    }
    else 
    {
	// destroy original pointers
	a = b = 0;
    }
}

struct ApplyEventHelper : public EventDispatcher
{
    const Event &event;
    Environment &env;
    bool lenient;

    ApplyEventHelper( const Event& event, Environment& env ) : event( event ), env( env ), lenient( true ) {}

    void itemAttached(EnvironmentItem *item)
    {
	if( item && (lenient || item == env.getRootNode()) )
	{
	    // item already exists, but we can just overwrite it
	    item->set( event.a.get() );

	    env.itemModified( item );
	}
	else
	{
	    // doesn't exist yet, so attach it
	    env.attachItem( event.a.get() );
	}
    };

    void itemModified(EnvironmentItem *item)
    {
	if( item )
	{
	    // for the time being copy the whole item
	    // TODO: implement partial updates
	    item->set( event.a.get() );
	}
	else if( lenient )
	{
	    // since it has not been present,
	    // just add the map 
	    env.attachItem( event.a.get() );
	}
	else
	{
	    throw std::runtime_error("Event could not be applied. Item does not exist in environment.");
	}
	
	env.itemModified( item );
    }

    void itemDetached(EnvironmentItem *item)
    {
	if( item && !(item == env.getRootNode()) )
	{
	    env.detachItem( item );
	}
	else if( !lenient )
	{
	    throw std::runtime_error("Event could not be applied. Item does not exist in environment.");
	}
    }

    void childAdded(FrameNode* parent, FrameNode* child)
    {
	if( parent == NULL || child == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.addChild( parent, child );
    }

    void childAdded(Layer* parent, Layer* child)
    {
	if( parent == NULL || child == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.addChild( parent, child );
    }

    void frameNodeSet(CartesianMap* map, FrameNode* node)
    {
	if( map == NULL || node == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.setFrameNode( map, node );
    }

    void frameNodeDetached(CartesianMap* map, FrameNode* node)
    {
	if( map == NULL || node == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.detachFrameNode( map, node );
    }

    void childRemoved(FrameNode* parent, FrameNode* child)
    {
	if( parent == NULL || child == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.removeChild( parent, child );
    }

    void childRemoved(Layer* parent, Layer* child)
    {
	if( parent == NULL || child == NULL )
	{
	    if( lenient )
		return;
	    else
		throw std::runtime_error("Event could not be applied. Items missing for applying addChild relation.");
	}

	env.removeChild( parent, child );
    }

    void setRootNode(FrameNode *root)
    {
	// the root node should stay the same for all environments
	// TODO: maybe improve handling of the root node
    }

    void removeRootNode(FrameNode *root)
    {
    }

};

void Event::apply( Environment* env ) const
{
    EnvironmentItem *a = env->getItem( id_a ).get();
    EnvironmentItem *b = env->getItem( id_b ).get();

    ApplyEventHelper helper( *this, *env );
    EventDispatcher::dispatch( type, operation, a, b, &helper );
}

