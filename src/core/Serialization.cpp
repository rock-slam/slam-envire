#include "Serialization.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <iterator>
#include <algorithm>

extern "C" {
#include <yaml.h>
}

using namespace std;
using namespace envire;
namespace fs = boost::filesystem;

namespace envire 
{
    class YAMLSerializationImpl
    {
    public:
        yaml_parser_t parser;
        yaml_emitter_t emitter;
        yaml_document_t document;

        int current_node;

        vector<yaml_char_t*> buffers;
        
        
        YAMLSerializationImpl();
        
        yaml_node_t* getNode(int index);

        std::string getScalar(int node_index);
        template<class T> T getScalarInMap(const std::string &key)
            { return boost::lexical_cast<T>( getScalar( findNodeInMap(key) ) ); };
        int findNodeInMap( int map_index, const std::string &key );
        int findNodeInMap( const std::string &key );

        bool addNodeToMap( const std::string &key, int value_index );
        bool addNodeToMap( int map_index, const std::string &key, int value_index );

        template<class T> int addScalar(const T &value, yaml_scalar_style_t style = YAML_SINGLE_QUOTED_SCALAR_STYLE);
        int addSequenceNode(yaml_sequence_style_t style = YAML_ANY_SEQUENCE_STYLE);
        int addMapNode(yaml_mapping_style_t style = YAML_ANY_MAPPING_STYLE);
        int addToSequence(int seq_id, int node_id);
    };

    template<>
	int YAMLSerializationImpl::addScalar( const std::string &value, yaml_scalar_style_t style )
	{
	    size_t len = value.length();

	    yaml_char_t* buf = new yaml_char_t[len];
	    value.copy(reinterpret_cast<char*>(buf), len);
	    buffers.push_back( buf );

	    int key_index = yaml_document_add_scalar( &document, NULL, buf, len, style );

	    return key_index;
	}

    template<class T>
	int YAMLSerializationImpl::addScalar( const T &v, yaml_scalar_style_t style )
	{
	    std::string value = boost::lexical_cast<std::string>(v);
	    return addScalar( value, YAML_PLAIN_SCALAR_STYLE );
	}
}

YAMLSerializationImpl::YAMLSerializationImpl()
{

}

bool YAMLSerializationImpl::addNodeToMap( const std::string &key, int value_index )
{
    addNodeToMap( current_node, key, value_index );

    return true;
}

bool YAMLSerializationImpl::addNodeToMap( int map_index, const std::string &key, int value_index )
{
    int key_index = addScalar( key, YAML_PLAIN_SCALAR_STYLE );
    yaml_document_append_mapping_pair(&document, map_index, key_index, value_index);

    return true;
}

int YAMLSerializationImpl::addSequenceNode(yaml_sequence_style_t style)
{
    return yaml_document_add_sequence(&document, NULL, style);
}

int YAMLSerializationImpl::addMapNode(yaml_mapping_style_t style)
{
    return yaml_document_add_mapping(&document, NULL, style);
}

int YAMLSerializationImpl::addToSequence(int seq_id, int node_id)
{
    return yaml_document_append_sequence_item( &document, seq_id, node_id );
}

int YAMLSerializationImpl::findNodeInMap( const std::string &key )
{
    return findNodeInMap( current_node, key );
}

int YAMLSerializationImpl::findNodeInMap( int map_index, const std::string &key )
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

yaml_node_t* YAMLSerializationImpl::getNode( int index )
{
    return yaml_document_get_node(&document, index);
}

std::string YAMLSerializationImpl::getScalar( int node_index )
{
    yaml_node_t* node = yaml_document_get_node(&document, node_index);

    if( node && node->type == YAML_SCALAR_NODE )
    {
        return std::string( reinterpret_cast<const char*>(node->data.scalar.value) );
    }
    throw std::runtime_error("not a scalar node");
}


//// Serialization ////

Serialization::Serialization()
    : yamlSerialization(new YAMLSerializationImpl) {}

Serialization::~Serialization()
{
    delete yamlSerialization;
}

void Serialization::write(const std::string& key, const std::string& value)
{
    yamlSerialization->addNodeToMap( key, yamlSerialization->addScalar(value) );
}

void Serialization::write(const std::string& key, const FrameNode::TransformType &value)
{
    // create a sequence node with all the elements
    int seq_id = yamlSerialization->addSequenceNode( YAML_FLOW_SEQUENCE_STYLE );
    yamlSerialization->addNodeToMap( key, seq_id );

    for(int i=0;i<value.matrix().rows();i++)
    {
        for(int j=0;j<value.matrix().cols();j++)
        {
            yamlSerialization->addToSequence( seq_id, yamlSerialization->addScalar( value.matrix()(i,j) ) );
        }
    }
}

void Serialization::begin()
{
    yaml_document_initialize(&yamlSerialization->document, NULL, NULL, NULL, 1, 1);
    int obj_id = yaml_document_add_sequence(&yamlSerialization->document, NULL, YAML_ANY_SEQUENCE_STYLE);
    yamlSerialization->current_node = yamlSerialization->addMapNode();
    yamlSerialization->addToSequence( obj_id, yamlSerialization->current_node );
}

void Serialization::end()
{
    yaml_document_delete(&yamlSerialization->document);
}

bool Serialization::hasKey(const std::string& key) const
{
    int node_index = yamlSerialization->findNodeInMap( key );
    return (node_index != 0);
}

bool Serialization::read(const std::string &key, std::string &value)
{
    try
    {
        value = yamlSerialization->getScalar( yamlSerialization->findNodeInMap( key ) );
        return true;
    }
    catch(...)
    {
        std::cerr << "WARN: could not read scalar for " << key << std::endl;
        return false;
    }
}

bool Serialization::read(const std::string& key, FrameNode::TransformType &value)
{
    int node_index = yamlSerialization->findNodeInMap( key );
    yaml_node_t* node = yamlSerialization->getNode( node_index );
    
    if( node->type != YAML_SEQUENCE_NODE )
        throw std::runtime_error("can't read TransformType");

    int i=0;
    for(yaml_node_item_t* item=node->data.sequence.items.start;
            item < node->data.sequence.items.top; item++)
    {
        std::string t(yamlSerialization->getScalar( *item ) );

        value( i / value.matrix().rows(), i % value.matrix().rows() ) =
            boost::lexical_cast<FrameNode::TransformType::Scalar>( t );
        i++;
    }
   
    if( i != value.matrix().rows() * value.matrix().cols() )
       throw std::runtime_error("matrix dimension incompatible");       

    return true;
}

std::istream& Serialization::getBinaryInputStream(const std::string &filename)
{
    throw NoSuchBinaryStream("trying to access a binary input stream on a plain Serialization object");
}

std::ostream& Serialization::getBinaryOutputStream(const std::string &filename)
{
    throw NoSuchBinaryStream("trying to access a binary output stream on a plain Serialization object");
}

//// FileSerialization ////

const std::string FileSerialization::STRUCTURE_FILE = "scene.yml";

FileSerialization::FileSerialization()
{
}

FileSerialization::~FileSerialization()
{
}

const std::string FileSerialization::getMapPath() const
{
    return sceneDir;
}

void FileSerialization::setSceneDir(const std::string dir)
{
    sceneDir = dir;
}

std::istream& FileSerialization::getBinaryInputStream(const std::string &filename)
{
    boost::filesystem::path fileDir(sceneDir);
    fileDir = fileDir / filename;
    std::ifstream *is = new std::ifstream(fileDir.string().c_str());
    if( !is->is_open() || is->fail() )
    {
        throw NoSuchBinaryStream("could not open file " + filename);
    }
    ifstreams.push_back(is);
    return *is;
}

std::ostream& FileSerialization::getBinaryOutputStream(const std::string &filename)
{
    boost::filesystem::path fileDir(sceneDir);
    fileDir = fileDir / filename;
    std::ofstream *os = new std::ofstream(fileDir.string().c_str());
    if( !os->is_open() || os->fail() )
    {
        throw NoSuchBinaryStream("could not open file " + filename);
    }
    ofstreams.push_back(os);
    return *os;
}

bool FileSerialization::writeToFile( Environment *env, const std::string &path )
{
    yaml_emitter_initialize(&yamlSerialization->emitter);
    FILE *output = fopen(path.c_str(), "wb");
    if( !output )
	throw runtime_error("envire: could not open " + path + " for writing");

    yaml_emitter_set_output_file(&yamlSerialization->emitter, output);

    // build up document
    if( !yaml_document_initialize(&yamlSerialization->document, NULL, NULL, NULL, 1, 1) )
    {
	yaml_emitter_delete(&yamlSerialization->emitter);
	fclose( output );
	throw std::runtime_error("could not generate yaml document");
    }
    
    // same as with readFile, creating a dom structure for new is easier.
    int obj_id, link_id, root_id;
    root_id = yaml_document_add_mapping(
	    &yamlSerialization->document, NULL, YAML_ANY_MAPPING_STYLE);

    obj_id = yaml_document_add_sequence(
	    &yamlSerialization->document, NULL, YAML_ANY_SEQUENCE_STYLE);

    link_id = yaml_document_add_sequence(
	    &yamlSerialization->document, NULL, YAML_ANY_SEQUENCE_STYLE);

    yamlSerialization->addNodeToMap( root_id, "objects", obj_id );
    yamlSerialization->addNodeToMap( root_id, "links", link_id );

    // dump all objects now
    for( Environment::itemListType::iterator it = env->items.begin();
	    it != env->items.end();it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( obj_id, yamlSerialization->current_node );

	yamlSerialization->addNodeToMap( "class", yamlSerialization->addScalar((*it).second->getClassName()) );
	(*it).second->serialize( *this );
    }

    // and all the links
    for( Environment::frameNodeTreeType::iterator it = env->frameNodeTree.begin();
	    it != env->frameNodeTree.end(); it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( link_id, yamlSerialization->current_node );
	
	yamlSerialization->addNodeToMap( "type", yamlSerialization->addScalar("frameNodeTree") );
	yamlSerialization->addNodeToMap( "child", yamlSerialization->addScalar((*it).first->getUniqueId()) );
	yamlSerialization->addNodeToMap( "parent", yamlSerialization->addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::layerTreeType::iterator it = env->layerTree.begin();
	    it != env->layerTree.end(); it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( link_id, yamlSerialization->current_node );
	
	yamlSerialization->addNodeToMap( "type", yamlSerialization->addScalar("layerTree") );
	yamlSerialization->addNodeToMap( "child", yamlSerialization->addScalar((*it).first->getUniqueId()) );
	yamlSerialization->addNodeToMap( "parent", yamlSerialization->addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::operatorGraphType::iterator it = env->operatorGraphInput.begin();
	    it != env->operatorGraphInput.end(); it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( link_id, yamlSerialization->current_node );
	
	yamlSerialization->addNodeToMap( "type", yamlSerialization->addScalar("operatorGraphInput") );
	yamlSerialization->addNodeToMap( "operator", yamlSerialization->addScalar((*it).first->getUniqueId()) );
	yamlSerialization->addNodeToMap( "layer", yamlSerialization->addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::operatorGraphType::iterator it = env->operatorGraphOutput.begin();
	    it != env->operatorGraphOutput.end(); it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( link_id, yamlSerialization->current_node );
	
	yamlSerialization->addNodeToMap( "type", yamlSerialization->addScalar("operatorGraphOutput") );
	yamlSerialization->addNodeToMap( "operator", yamlSerialization->addScalar((*it).first->getUniqueId()) );
	yamlSerialization->addNodeToMap( "layer", yamlSerialization->addScalar((*it).second->getUniqueId()) );
    }

    for( Environment::cartesianMapGraphType::iterator it = env->cartesianMapGraph.begin();
	    it != env->cartesianMapGraph.end(); it++ )
    {
	yamlSerialization->current_node = yamlSerialization->addMapNode();
	yamlSerialization->addToSequence( link_id, yamlSerialization->current_node );
	
	yamlSerialization->addNodeToMap( "type", yamlSerialization->addScalar("cartesianMapGraph") );
	yamlSerialization->addNodeToMap( "map", yamlSerialization->addScalar((*it).first->getUniqueId()) );
	yamlSerialization->addNodeToMap( "node", yamlSerialization->addScalar((*it).second->getUniqueId()) );
    }

    // the emitter will destroy the document objects for us.
    // what it will not do however, is destroy the buffers for the scalar values.
    // (damn... maybe using an object was not the smartest choice. Events would
    // have done just as fine... next time)
    // So what we have to do is extract a list of all the scalars here and free them
    // afterwards

    int result = yaml_emitter_dump( &yamlSerialization->emitter, &yamlSerialization->document );

    for(vector<yaml_char_t*>::iterator it=yamlSerialization->buffers.begin();it<yamlSerialization->buffers.end();it++)
    {
	delete[] *it;
    }
    yamlSerialization->buffers.clear();

    yaml_emitter_delete(&yamlSerialization->emitter);
    yaml_document_delete(&yamlSerialization->document);
    fclose( output );
    
    // close and delete ofstreams
    if(ofstreams.size() > 0)
    {
        for(std::vector<std::ofstream*>::iterator it = ofstreams.begin(); it != ofstreams.end(); it++)
        {
            if((*it)->is_open())
            {
                (*it)->close();
            }
            delete *it;
        }
        ofstreams.clear();
    }
    
    return result;
}

template<typename MapType>
static MapType* getMap(YAMLSerializationImpl* yaml, Environment* env, const char* key)
{
    std::string map_id = yaml->getScalarInMap<std::string>(key);
    if (*map_id.begin() != '/')
        map_id = "/" + map_id;

    MapType* map = env->getItem<MapType>(map_id).get();
    if (!map)
        throw std::runtime_error("cannot find map " + map_id + " in scene");
    return map;
}

Environment* FileSerialization::readFromFile( const std::string& path )
{
    Environment* env;
    
    int64_t lastID = 0;

    FILE *input = fopen(path.c_str(), "rb");
    yaml_parser_initialize(&yamlSerialization->parser);
    yaml_parser_set_input_file(&yamlSerialization->parser, input);

    // for now lets ignore SAX-like streaming, and parse the whole
    // document into a document structure. This makes it a little easier
    // to extract.

    if(!yaml_parser_load(&yamlSerialization->parser, &yamlSerialization->document))
    {
	// doubled the cleanup code... not nice.
	yaml_parser_delete(&yamlSerialization->parser);
	fclose(input);

	std::cerr << path << ":" 
	    << yamlSerialization->parser.problem_mark.line << ":"
	    << yamlSerialization->parser.problem;
	throw std::runtime_error("error parsing yaml stream.");
    }

    // do the extraction now
    yaml_node_t* root = yaml_document_get_root_node(&yamlSerialization->document);

    if( root && root->type == YAML_MAPPING_NODE )
    {
	// create a new environment
	env = new Environment();
	// and remove the root node
	EnvironmentItem* rootNode = env->getRootNode();
	env->detachItem( rootNode );

	// browse root mapping for object or links	
	for(yaml_node_pair_t* pair=root->data.mapping.pairs.start;
		pair < root->data.mapping.pairs.top; pair++)
	{
	    std::string key = yamlSerialization->getScalar( pair->key );
	    yaml_node_t* list = yaml_document_get_node(&yamlSerialization->document, pair->value );

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
			std::string className = yamlSerialization->getScalar( yamlSerialization->findNodeInMap( *item, "class" ) );
			
			// store current state in class
			yamlSerialization->current_node = *item;

			// create item and attach to environment
			EnvironmentItem* envItem = 
			    SerializationFactory::createObject(className, *this);

			env->attachItem( envItem );
			//hack to preserve root node
			if(envItem->getUniqueId() == "0" || envItem->getUniqueId() == "/0") {
			    env->rootNode = dynamic_cast<FrameNode *>(envItem);
			    assert(env->rootNode);
			}
			
                        std::string currentID = envItem->getUniqueIdSuffix();
                        // If this is a number, make sure that we update lastID
                        // properly
                        try
                        {
                            long id = boost::lexical_cast<long>(currentID);
                            if(id > lastID)
                                lastID = id;
                        }
                        catch(boost::bad_lexical_cast)
                        { }
		    }
		    else if( key == "links" )
		    {
			yamlSerialization->current_node = *item;

			if( yamlSerialization->getScalarInMap<std::string>("type") == "frameNodeTree" )
			{
			    env->frameNodeTree.insert( make_pair( 
					getMap<FrameNode>(yamlSerialization, env, "child"), 
					getMap<FrameNode>(yamlSerialization, env, "parent") ) );
			}

			if( yamlSerialization->getScalarInMap<std::string>("type") == "layerTree" )
			{
			    env->layerTree.insert( make_pair( 
                                        getMap<Layer>(yamlSerialization, env, "child"),
                                        getMap<Layer>(yamlSerialization, env, "parent") ));
			}

			if( yamlSerialization->getScalarInMap<std::string>("type") == "operatorGraphInput" )
			{
			    env->operatorGraphInput.insert( make_pair( 
                                        getMap<Operator>(yamlSerialization, env, "operator"),
                                        getMap<Layer>(yamlSerialization, env, "layer")) );
			}

			if( yamlSerialization->getScalarInMap<std::string>("type") == "operatorGraphOutput" )
			{
			    env->operatorGraphOutput.insert( make_pair( 
                                        getMap<Operator>(yamlSerialization, env, "operator"),
                                        getMap<Layer>(yamlSerialization, env, "layer")) );
			}

			if( yamlSerialization->getScalarInMap<std::string>("type") == "cartesianMapGraph" )
			{
			    env->cartesianMapGraph.insert( make_pair( 
                                        getMap<CartesianMap>(yamlSerialization, env, "map"),
                                        getMap<FrameNode>(yamlSerialization, env, "node")) );
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

    lastID++;
    env->last_id = lastID;

    // clean up
    yaml_document_delete(&yamlSerialization->document);
    yaml_parser_delete(&yamlSerialization->parser);
    fclose(input);
    
    // close and delete ifstreams
    if(ifstreams.size() > 0)
    {
        for(std::vector<std::ifstream*>::iterator it = ifstreams.begin(); it != ifstreams.end(); it++)
        {
            if((*it)->is_open())
            {
                (*it)->close();
            }
            delete *it;
        }
        ifstreams.clear();
    }

    return env;
}


//// BinarySerialization ////

BinarySerialization::BinarySerialization()
{
}

BinarySerialization::~BinarySerialization()
{
}

std::istream& BinarySerialization::getBinaryInputStream(const std::string &filename)
{
    if(stringstreams[filename] == 0)
        throw NoSuchBinaryStream("there is no binary input stream called " + filename);
    return *stringstreams[filename];
}

std::ostream& BinarySerialization::getBinaryOutputStream(const std::string &filename)
{
    std::stringstream* ostream = new std::stringstream();
    stringstreams[filename] = ostream;
    return *ostream;
}

void BinarySerialization::applyEvents(envire::Environment* env,
        const std::vector<EnvireBinaryEvent>& events)
{
    BinarySerialization serialization;
    for (int i = 0; i < events.size(); ++i)
        serialization.applyEvent(env, events[i]);
}

void BinarySerialization::applyEvent(envire::Environment* env, const EnvireBinaryEvent& binary_event)
{
    EnvironmentItem* item = 0;
    if(binary_event.type == event::ITEM && (binary_event.operation == event::ADD || binary_event.operation == event::UPDATE ))
    {
        // unserialize item
        item = unserializeBinaryEvent(binary_event);
    }
    
    // set up event
    EnvironmentItem::Ptr item_ptr(item);
    envire::Event event(binary_event.type, binary_event.operation, item_ptr);
    event.id_a = binary_event.id_a;
    event.id_b = binary_event.id_b;
    
    // apply event
    event.apply(env);
}

EnvironmentItem* BinarySerialization::unserializeBinaryEvent(const EnvireBinaryEvent& bin_item)
{
    // set up yaml document
    yaml_parser_initialize(&yamlSerialization->parser);
    
    yaml_parser_set_input_string(&yamlSerialization->parser, reinterpret_cast<const unsigned char*>(bin_item.yamlProperties.data()), bin_item.yamlProperties.size());
    if(!yaml_parser_load(&yamlSerialization->parser, &yamlSerialization->document))
    {
        yaml_parser_delete(&yamlSerialization->parser);

        std::cerr << bin_item.className << " " << bin_item.id_a << ":" 
            << yamlSerialization->parser.problem_mark.line << ":"
            << yamlSerialization->parser.problem;
        throw std::runtime_error("error parsing yaml stream.");
    }
    
    // set up yaml node
    yamlSerialization->current_node = -1;
    yaml_node_t* root = yaml_document_get_root_node(&yamlSerialization->document);
    for(yaml_node_item_t* item=root->data.sequence.items.start;
            item < root->data.sequence.items.top; item++)
    {
         std::string className = yamlSerialization->getScalar( yamlSerialization->findNodeInMap( *item, "class" ) );
         if(className == bin_item.className)
         {
             yamlSerialization->current_node = *item;
             break;
         }
    }
    if(yamlSerialization->current_node < 0)
    {
        throw std::runtime_error("can't find class information in yaml stream.");
    }
    
    // set up binary streams
    for(unsigned int i = 0; i < bin_item.binaryStreamNames.size(); i++)
    {
        if(bin_item.binaryStreams.size() > i)
        {
            std::stringstream *istream = new std::stringstream();
            const std::vector<uint8_t> &bin_stream = bin_item.binaryStreams[i];
            std::copy(bin_stream.begin(), bin_stream.end(), ostream_iterator<uint8_t>(*istream));
            stringstreams[bin_item.binaryStreamNames[i]] = istream;
        }
    }
    
    // unserialize envire item
    EnvironmentItem* item = SerializationFactory::createObject(bin_item.className, *this);
    
    //clean up
    yaml_document_delete(&yamlSerialization->document);
    yaml_parser_delete(&yamlSerialization->parser);
    cleanUp();
    
    return item;
}

bool BinarySerialization::serializeBinaryEvent(EnvironmentItem* item, EnvireBinaryEvent& bin_item)
{
    assert(item);
    
    bin_item.className = item->getClassName();
    bin_item.yamlProperties.clear();
    bin_item.binaryStreamNames.clear();
    bin_item.binaryStreams.clear();
    
    // config yaml
    yaml_emitter_initialize(&yamlSerialization->emitter);
    bin_item.yamlProperties.resize(10000,0);
    size_t size_written = 0;
    yaml_emitter_set_output_string(&yamlSerialization->emitter, reinterpret_cast<unsigned char*>(bin_item.yamlProperties.data()),
                                   bin_item.yamlProperties.size(), &size_written);
    
    // build up document
    if( !yaml_document_initialize(&yamlSerialization->document, NULL, NULL, NULL, 1, 1) )
    {
        yaml_emitter_delete(&yamlSerialization->emitter);
        throw std::runtime_error("could not generate yaml document");
    }
    
    // write document
    int obj_id = yaml_document_add_sequence(&yamlSerialization->document, NULL, YAML_ANY_SEQUENCE_STYLE);
    yamlSerialization->current_node = yamlSerialization->addMapNode();
    yamlSerialization->addToSequence( obj_id, yamlSerialization->current_node );
    yamlSerialization->addNodeToMap( "class", yamlSerialization->addScalar( item->getClassName() ));
    
    // serialize envire item
    item->serialize(*this);
    
    // write yaml document to yaml stream
    int result = yaml_emitter_dump( &yamlSerialization->emitter, &yamlSerialization->document );
    bin_item.yamlProperties.resize(size_written,0);
    
    // fill binary streams
    for(std::map<std::string, std::stringstream*>::iterator it = stringstreams.begin(); it != stringstreams.end(); it++)
    {
        std::vector<uint8_t> bin_vector;
        std::stringstream& ostream = *it->second;
        ostream >> std::noskipws;
        std::copy(istream_iterator<uint8_t>(ostream), istream_iterator<uint8_t>(), std::back_inserter(bin_vector));
        bin_item.binaryStreamNames.push_back(it->first);
        bin_item.binaryStreams.push_back(bin_vector);
    }

    // clean up
    for(vector<yaml_char_t*>::iterator it=yamlSerialization->buffers.begin();it<yamlSerialization->buffers.end();it++)
    {
        delete[] *it;
    }
    yamlSerialization->buffers.clear();
    yaml_emitter_delete(&yamlSerialization->emitter);
    yaml_document_delete(&yamlSerialization->document);
    cleanUp();
    
    return result;
}

void BinarySerialization::cleanUp()
{
    // delete stringstreams
    for(std::map<std::string, std::stringstream*>::iterator it = stringstreams.begin(); it != stringstreams.end(); it++)
    {
        delete it->second;
    }
    stringstreams.clear();
}


//// SynchronizationEventHandler ////

void SynchronizationEventHandler::handle(const envire::Event& message)
{
    if(use_event_queue)
    {
        EventQueue::handle(message);
    }
    else
    {
        if(msgQueue.size())
            flush();
        process(message);
    }
}

void SynchronizationEventHandler::process(const envire::Event& message)
{
    // if there is a filter, see if the event gets filtered out
    if( filter && !filter->filter( message ) )
        return;

    std::string id_a = message.a ? message.a->getUniqueId() : message.id_a;
    std::string id_b = message.b ? message.b->getUniqueId() : message.id_b;
    
    EnvireBinaryEvent* binary_event = new EnvireBinaryEvent(message.type, message.operation, id_a, id_b);
    
    if(message.type == event::ITEM && ( message.operation == event::ADD || message.operation == event::UPDATE ))
    {
        serialization.serializeBinaryEvent(message.a.get(), *binary_event);
    }
    
    handle(binary_event);
}

void SynchronizationEventHandler::useEventQueue(bool b)
{
    use_event_queue = b;
}

