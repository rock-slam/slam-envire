#include "Core.hpp"
#include "LaserScan.hpp"

#include <boost/assign/list_of.hpp>
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/lexical_cast.hpp"

#include <iostream>

extern "C" {
#include <yaml.h>
}

using namespace std;
using namespace envire;
using namespace boost::assign;
namespace fs = boost::filesystem;

namespace envire 
{
    class SerializationImpl
    {
    protected:
	Serialization& so;

	yaml_parser_t parser;
	yaml_event_t event;
	yaml_document_t document;

	int current_node;

    public:
	SerializationImpl(Serialization &so);
	Environment* readFromFile(const std::string &path);

	yaml_node_t* getNode(int index);

	std::string getScalar(int node_index);
	int findNodeInMap( int map_index, const std::string &key );
	int findNodeInMap( const std::string &key );
	bool addNodeToMap( const std::string &key, int value_index );
	bool addNodeToMap( const std::string &key, const std::string &value );
	bool addNodeToMap( int map_index, const std::string &key, int value_index );
    };
}


const std::string Serialization::STRUCTURE_FILE = "scene.yml";

template<class T> EnvironmentItem* create(Serialization &so) 
{
    T* o = new T(so);
    return o;
}

map<std::string, Serialization::Factory> Serialization::classMap 
    = map_list_of("envire::FrameNode", &create<FrameNode> );


Serialization::Serialization()
{
    impl = new SerializationImpl( *this );
}

Serialization::~Serialization()
{
    delete impl;
}

void Serialization::write(const std::string& key, const std::string& value)
{
}

void Serialization::write(const std::string& key, long value)
{
}

void Serialization::write(const std::string& key, const FrameNode::TransformType &value)
{

}

void Serialization::read(const std::string &key, std::string &value)
{
    value = impl->getScalar( impl->findNodeInMap( key ) );
}

void Serialization::read(const std::string &key, long &value)
{
    value = boost::lexical_cast<long>( impl->getScalar( impl->findNodeInMap( key ) ) );
}

void Serialization::read(const std::string& key, FrameNode::TransformType &value)
{
    int node_index = impl->findNodeInMap( key );
    yaml_node_t* node = impl->getNode( node_index );
    
    if( node->type != YAML_SEQUENCE_NODE )
	throw std::runtime_error("can't read TransformType");

    int i=0;
    for(yaml_node_item_t* item=node->data.sequence.items.start;
	    item < node->data.sequence.items.top; item++)
    {
	std::string t(impl->getScalar( *item ) );

	value( i / value.matrix().rows(), i % value.matrix().rows() ) =
	    boost::lexical_cast<FrameNode::TransformType::Scalar>( t );
	i++;
    }
   
    if( i != value.matrix().rows() * value.matrix().cols() )
       throw std::runtime_error("matrix dimension incompatible");	
}

void Serialization::serialize(Environment *env, const std::string &path_str) 
{
        
}

Environment* Serialization::unserialize(const std::string &path_str) 
{
    fs::path path( path_str ); 
    fs::path scene( path / STRUCTURE_FILE );

    if( !fs::is_regular( scene ) )
    {
	std::cerr << "failed to open " << scene << std::endl;
	throw std::runtime_error("Could not open file");
    }

    return impl->readFromFile( scene.string() );
}


SerializationImpl::SerializationImpl(Serialization &_so)
    : so( _so ) 
{
}

Environment* SerializationImpl::readFromFile( const std::string& path )
{
    Environment* env;

    FILE *input = fopen(path.c_str(), "rb");
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_file(&parser, input);

    // for now lets ignore SAX-like streaming, and parse the whole
    // document into a document structure. This makes it a little easier
    // to extract.

    if(!yaml_parser_load(&parser, &document))
    {
	// doubled the cleanup code... not nice.
	yaml_parser_delete(&parser);
	fclose(input);

	std::cerr << path << ":" 
	    << parser.problem_mark.line << ":"
	    << parser.problem;
	throw std::runtime_error("error parsing yaml stream.");
    }

    // do the extraction now
    yaml_node_t* root = yaml_document_get_root_node(&document);

    if( root && root->type == YAML_MAPPING_NODE )
    {
	// create a new environment
	env = new Environment();

	// browse root mapping for object or links	
	for(yaml_node_pair_t* pair=root->data.mapping.pairs.start;
		pair < root->data.mapping.pairs.top; pair++)
	{
	    std::string key = getScalar( pair->key );
	    yaml_node_t* list = yaml_document_get_node(&document, pair->value );

	    if( list->type == YAML_SEQUENCE_NODE )
	    {
		// loop through items in the sequence
		for(yaml_node_item_t* item=list->data.sequence.items.start;
			item < list->data.sequence.items.top; item++)
		{
		    if( key == "objects" )
		    {
			// this is an object node. 
			// find the classname first
			std::string className = getScalar( findNodeInMap( *item, "class" ) );
			
			// store current state in class
			current_node = *item;

			Serialization::Factory f = 0;
			f = Serialization::classMap[className];
			if( f )
			{
			    (*f)(so);
			}	    
			else 
			{
			    std::cerr << "could not find class of type " << className << std::endl;
			    throw std::runtime_error("could not find class");
			}
		    }
		    else if( key == "links" )
		    {
		    }
		}
	    }
	}

    }
    else 
    {
	throw std::runtime_error("empty document.");
    }

    // clean up
    yaml_document_delete(&document);
    yaml_parser_delete(&parser);
    fclose(input);

    return env;
}

bool SerializationImpl::addNodeToMap( const std::string &key, int value_index )
{
    addNodeToMap( current_node, key, value_index );
}

bool SerializationImpl::addNodeToMap( const std::string &key, const std::string &value )
{
    yaml_char_t* buf = new yaml_char_t[value.length()];
    value.copy(reinterpret_cast<char*>(buf), value.length());

    int index = yaml_document_add_scalar( &document, NULL,
	    buf, value.length(),
	    YAML_ANY_SCALAR_STYLE ); 
    addNodeToMap( current_node, key, index ); 
}

bool SerializationImpl::addNodeToMap( int map_index, const std::string &key, int value_index )
{
    yaml_char_t* buf = new yaml_char_t[key.length()];
    key.copy(reinterpret_cast<char*>(buf), key.length());

    int key_index = yaml_document_add_scalar( &document, NULL, buf, key.length(), YAML_ANY_SCALAR_STYLE );
    yaml_document_append_mapping_pair(&document, map_index, key_index, value_index);
}

int SerializationImpl::findNodeInMap( const std::string &key )
{
    return findNodeInMap( current_node, key );
}

int SerializationImpl::findNodeInMap( int map_index, const std::string &key )
{
    yaml_node_t *map = getNode( map_index );

    assert( map->type == YAML_MAPPING_NODE );

    int node = 0;

    for(yaml_node_pair_t* pair=map->data.mapping.pairs.start;
	    pair < map->data.mapping.pairs.top; pair++)
    {
	std::string _key = getScalar( pair->key );

	if( _key == key )
	{
	    if( node )
		std::cerr << "found more than one key " << key << " for the mapping. This is almost certainly bad!" << std::endl;

	    node = pair->value;
	}
    }

    return node;
}

yaml_node_t* SerializationImpl::getNode( int index )
{
    return yaml_document_get_node(&document, index);
}

std::string SerializationImpl::getScalar( int node_index )
{
    yaml_node_t* node = yaml_document_get_node(&document, node_index);

    if( node && node->type == YAML_SCALAR_NODE )
    {
	return std::string( reinterpret_cast<const char*>(node->data.scalar.value) );
    }

    return NULL;
}


