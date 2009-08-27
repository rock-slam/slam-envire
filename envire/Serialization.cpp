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
	yaml_emitter_t emitter;
	yaml_event_t event;
	yaml_document_t document;

	int current_node;
	std::string className;
	bool serialize;

    public:
	SerializationImpl(Serialization &so);
	Environment* readFromFile(const std::string &path);
	bool writeToFile( Environment* env, const std::string &path );

	yaml_node_t* getNode(int index);

	std::string getScalar(int node_index);
	template<class T> T getScalarInMap(const std::string &key)
	    { return boost::lexical_cast<T>( getScalar( findNodeInMap(key) ) ); };
	int findNodeInMap( int map_index, const std::string &key );
	int findNodeInMap( const std::string &key );

	void setClassName( const std::string &key );
	bool addNodeToMap( const std::string &key, int value_index );
	bool addNodeToMap( int map_index, const std::string &key, int value_index );

	template<class T> int addScalar(const T &value, yaml_scalar_style_t style = YAML_SINGLE_QUOTED_SCALAR_STYLE);
	int addSequenceNode(yaml_sequence_style_t style = YAML_ANY_SEQUENCE_STYLE);
	int addMapNode(yaml_mapping_style_t style = YAML_ANY_MAPPING_STYLE);
	int addToSequence(int seq_id, int node_id);
    };

    template<>
	int SerializationImpl::addScalar( const std::string &value, yaml_scalar_style_t style )
	{
	    yaml_char_t* buf = new yaml_char_t[value.length()];
	    value.copy(reinterpret_cast<char*>(buf), value.length());

	    int key_index = yaml_document_add_scalar( &document, NULL, buf, value.length(), style );

	    return key_index;
	}

    template<class T>
	int SerializationImpl::addScalar( const T &v, yaml_scalar_style_t style )
	{
	    std::string value = boost::lexical_cast<std::string>(v);
	    return addScalar( value, YAML_PLAIN_SCALAR_STYLE );
	}
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

void Serialization::setClassName(const std::string &key)
{
    impl->setClassName(key);
}

void Serialization::write(const std::string& key, const std::string& value)
{
    impl->addNodeToMap( key, impl->addScalar(value) );
}

void Serialization::write(const std::string& key, long value)
{
    impl->addNodeToMap( key, impl->addScalar(value) );
}

void Serialization::write(const std::string& key, const FrameNode::TransformType &value)
{
    // create a sequence node with all the elements
    int seq_id = impl->addSequenceNode( YAML_FLOW_SEQUENCE_STYLE );
    impl->addNodeToMap( key, seq_id );

    for(int i=0;i<value.matrix().rows();i++)
    {
	for(int j=0;j<value.matrix().cols();j++)
	{
	    impl->addToSequence( seq_id, impl->addScalar( value.matrix()(i,j) ) );
	}
    }
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
    fs::path path( path_str ); 
    fs::path scene( path / STRUCTURE_FILE );

    if( !fs::is_directory( path ) )
    {
	std::cerr << "is not a directory " << path_str << std::endl;
	throw std::runtime_error("Path is not a directory");
    }

    impl->writeToFile( env, scene.string() );
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

void SerializationImpl::setClassName(const std::string &key)
{
    // for now only store the top level class name in the hierarchy
    // also note, this call can arrive from serialisation and deserialisation
    if( serialize && className.empty() )
    {
	className = key;
	addNodeToMap( "class", addScalar(key) );
    }
}

bool SerializationImpl::writeToFile( Environment *env, const std::string &path )
{
    serialize = true;

    yaml_emitter_initialize(&emitter);
    FILE *output = fopen(path.c_str(), "wb");
    if( !output )
	throw runtime_error("could not open file for writing");

    yaml_emitter_set_output_file(&emitter, output);

    // build up document
    if( !yaml_document_initialize(&document, NULL, NULL, NULL, 1, 1) )
    {
	fclose( output );
	throw std::runtime_error("could not generate yaml document");
    }
    
    // same as with readFile, creating a dom structure for new is easier.
    int obj_id, link_id, root_id;
    root_id = yaml_document_add_mapping(
	    &document, NULL, YAML_ANY_MAPPING_STYLE);

    obj_id = yaml_document_add_sequence(
	    &document, NULL, YAML_ANY_SEQUENCE_STYLE);

    link_id = yaml_document_add_sequence(
	    &document, NULL, YAML_ANY_SEQUENCE_STYLE);

    addNodeToMap( root_id, "objects", obj_id );
    addNodeToMap( root_id, "links", link_id );

    // dump all objects now
    for( Environment::itemListType::iterator it = env->items.begin();
	    it != env->items.end();it++ )
    {
	current_node = addMapNode();
	addToSequence( obj_id, current_node );

	className.clear();
	(*it).second->serialize( so );
    }

    // and all the links
    for( Environment::frameNodeTreeType::iterator it = env->frameNodeTree.begin();
	    it != env->frameNodeTree.end(); it++ )
    {
	current_node = addMapNode();
	addToSequence( link_id, current_node );
	
	addNodeToMap( "type", addScalar("frameNodeTree") );
	addNodeToMap( "child", addScalar((*it).first->getUniqueId()) );
	addNodeToMap( "parent", addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::layerTreeType::iterator it = env->layerTree.begin();
	    it != env->layerTree.end(); it++ )
    {
	current_node = addMapNode();
	addToSequence( link_id, current_node );
	
	addNodeToMap( "type", addScalar("layerTree") );
	addNodeToMap( "child", addScalar((*it).first->getUniqueId()) );
	addNodeToMap( "parent", addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::operatorGraphType::iterator it = env->operatorGraphInput.begin();
	    it != env->operatorGraphInput.end(); it++ )
    {
	current_node = addMapNode();
	addToSequence( link_id, current_node );
	
	addNodeToMap( "type", addScalar("operatorGraphInput") );
	addNodeToMap( "operator", addScalar((*it).first->getUniqueId()) );
	addNodeToMap( "layer", addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::operatorGraphType::iterator it = env->operatorGraphOutput.begin();
	    it != env->operatorGraphOutput.end(); it++ )
    {
	current_node = addMapNode();
	addToSequence( link_id, current_node );
	
	addNodeToMap( "type", addScalar("operatorGraphOutput") );
	addNodeToMap( "operator", addScalar((*it).first->getUniqueId()) );
	addNodeToMap( "layer", addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::cartesianMapGraphType::iterator it = env->cartesianMapGraph.begin();
	    it != env->cartesianMapGraph.end(); it++ )
    {
	current_node = addMapNode();
	addToSequence( link_id, current_node );
	
	addNodeToMap( "type", addScalar("cartesianMapGraph") );
	addNodeToMap( "map", addScalar((*it).first->getUniqueId()) );
	addNodeToMap( "node", addScalar((*it).second->getUniqueId()) );
    }

    int result = yaml_emitter_dump( &emitter, &document );

    fclose( output );
    return result;
}

Environment* SerializationImpl::readFromFile( const std::string& path )
{
    serialize = false;

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
			current_node = *item;

			if( getScalarInMap<std::string>("type") == "frameNodeTree" )
			{
			    env->frameNodeTree.insert( make_pair( 
					env->getItem<FrameNode*>( getScalarInMap<long>("child") ), 
					env->getItem<FrameNode*>( getScalarInMap<long>("parent") ) ) );
			}

			if( getScalarInMap<std::string>("type") == "layerTree" )
			{
			    env->layerTree.insert( make_pair( 
					env->getItem<Layer*>( getScalarInMap<long>("child") ), 
					env->getItem<Layer*>( getScalarInMap<long>("parent") ) ) );
			}

			if( getScalarInMap<std::string>("type") == "operatorGraphInput" )
			{
			    env->operatorGraphInput.insert( make_pair( 
					env->getItem<Operator*>( getScalarInMap<long>("operator") ), 
					env->getItem<Layer*>( getScalarInMap<long>("layer") ) ) );
			}

			if( getScalarInMap<std::string>("type") == "operatorGraphOutput" )
			{
			    env->operatorGraphOutput.insert( make_pair( 
					env->getItem<Operator*>( getScalarInMap<long>("operator") ), 
					env->getItem<Layer*>( getScalarInMap<long>("layer") ) ) );
			}

			if( getScalarInMap<std::string>("type") == "cartesianMapGraph" )
			{
			    env->cartesianMapGraph.insert( make_pair( 
					env->getItem<CartesianMap*>( getScalarInMap<long>("map") ), 
					env->getItem<FrameNode*>( getScalarInMap<long>("node") ) ) );
			}
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

bool SerializationImpl::addNodeToMap( int map_index, const std::string &key, int value_index )
{
    int key_index = addScalar( key, YAML_PLAIN_SCALAR_STYLE );
    yaml_document_append_mapping_pair(&document, map_index, key_index, value_index);
}

int SerializationImpl::addSequenceNode(yaml_sequence_style_t style)
{
    return yaml_document_add_sequence(&document, NULL, style);
}

int SerializationImpl::addMapNode(yaml_mapping_style_t style)
{
    return yaml_document_add_mapping(&document, NULL, style);
}

int SerializationImpl::addToSequence(int seq_id, int node_id)
{
    yaml_document_append_sequence_item( &document, seq_id, node_id );
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

    throw std::runtime_error("not a scalar node");
}


