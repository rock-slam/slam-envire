#include "Core.hpp"
#include "core/Event.hpp"
#include "core/EventHandler.hpp"

using namespace envire;

std::ostream& envire::operator <<( std::ostream& ostream, Event::Type type )
{
    switch( type )
    {
	case Event::ITEM: ostream << "ITEM"; break;
	case Event::FRAMENODE_TREE: ostream << "FRAMENODE_TREE"; break;
	case Event::FRAMENODE: ostream << "FRAMENODE"; break;
	case Event::LAYER_TREE: ostream << "LAYER_TREE"; break;
	case Event::ROOT: ostream << "ROOT"; break;
    }
    return ostream;
}

std::ostream& envire::operator <<( std::ostream& ostream, Event::Operation operation )
{
    switch( operation )
    {
	case Event::ADD: ostream << "ADD"; break;
	case Event::REMOVE: ostream << "REMOVE"; break;
	case Event::UPDATE: ostream << "UPDATE"; break;
    }
    return ostream;
}

std::ostream& envire::operator <<( std::ostream& ostream, Event::Result result )
{
    switch( result )
    {
	case Event::IGNORE: ostream << "IGNORE"; break;
	case Event::INVALIDATE: ostream << "INVALIDATE"; break;
	case Event::CANCEL: ostream << "CANCEL"; break;
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

Event::Event( Type type, Operation operation, EnvironmentItem::Ptr a, EnvironmentItem::Ptr b  )
    : type(type), operation(operation), a(a), b(b), id_a( Environment::ITEM_NOT_ATTACHED ), id_b( Environment::ITEM_NOT_ATTACHED )
{
}

Event::Result Event::merge( const Event& other )
{
    if( operation == UPDATE )
    {
	return (type == other.type && operation == other.operation && a == other.a && b == other.b) ? INVALIDATE : IGNORE;
    }
    if( operation == REMOVE )
    {
	switch( type )
	{
	    case FRAMENODE_TREE: 
	    case FRAMENODE:
	    case LAYER_TREE:
		return (type == other.type && a == other.a && b == other.b) ? CANCEL : IGNORE;
	    case ITEM: 
		if( (a == other.a || a == other.b) && (other.operation == ADD || other.operation == UPDATE) )
		    return (type == other.type && other.operation == ADD) ? CANCEL : INVALIDATE;
		else
		    return IGNORE;
	    case ROOT:
		return (type == other.type && a == other.a) ? CANCEL : IGNORE;
	}
    }

    return IGNORE;
}

void Event::ref()
{
    // store unique id's
    if( a ) id_a = a->getUniqueId();
    if( b ) id_b = b->getUniqueId();

    if( type == Event::ITEM && ( operation == Event::ADD || operation == Event::UPDATE ) )
    {
	// perform a copy of the EnvironmentItem in these cases
	std::cout << *this << " " << typeid( a.get() ).name() << std::endl;
	a = a->clone();
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

    ApplyEventHelper( const Event& event, Environment& env ) : event( event ), env( env ) {}

    void itemAttached(EnvironmentItem *item)
    {
	env.attachItem( event.a.get() );
    };

    void itemModified(EnvironmentItem *item)
    {
	// for the time being copy the whole item
	// TODO: implement partial updates
	item->set( event.a.get() );
	
	env.itemModified( item );
    }

    void itemDetached(EnvironmentItem *item)
    {
	env.detachItem( item );
    }

    void childAdded(FrameNode* parent, FrameNode* child)
    {
	env.addChild( parent, child );
    }

    void childAdded(Layer* parent, Layer* child)
    {
	env.addChild( parent, child );
    }

    void frameNodeSet(CartesianMap* map, FrameNode* node)
    {
	env.setFrameNode( map, node );
    }

    void frameNodeDetached(CartesianMap* map, FrameNode* node)
    {
	env.detachFrameNode( map, node );
    }

    void childRemoved(FrameNode* parent, FrameNode* child)
    {
	env.removeChild( parent, child );
    }

    void childRemoved(Layer* parent, Layer* child)
    {
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

