#include "Core.hpp"
#include "core/Event.hpp"

using namespace envire;

std::ostream& operator <<( std::ostream& ostream, Event::Type type )
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

std::ostream& operator <<( std::ostream& ostream, Event::Operation operation )
{
    switch( operation )
    {
	case Event::ADD: ostream << "ADD"; break;
	case Event::REMOVE: ostream << "REMOVE"; break;
	case Event::UPDATE: ostream << "UPDATE"; break;
    }
    return ostream;
}

std::ostream& operator <<( std::ostream& ostream, Event::Result result )
{
    switch( result )
    {
	case Event::IGNORE: ostream << "IGNORE"; break;
	case Event::INVALIDATE: ostream << "INVALIDATE"; break;
	case Event::CANCEL: ostream << "CANCEL"; break;
    }
    return ostream;
}

std::ostream& operator <<( std::ostream& ostream, const Event& msg )
{
    ostream << "(" << msg.operation << " " << msg.type << " a:" << msg.a << " b:" << msg.b << ")";
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

    // destroy original pointers
    a = b = 0;

    if( type == Event::ITEM && ( operation == Event::ADD || operation == Event::UPDATE ) )
    {
	// perform a copy of the EnvironmentItem in these cases
	a = a->clone();
    }
}

void Event::apply( Environment* env )
{
}

