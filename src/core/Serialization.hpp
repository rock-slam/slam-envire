#ifndef __ENVIRE_SERIALIZATION_HPP__
#define __ENVIRE_SERIALIZATION_HPP__

#include <envire/Core.hpp>
#include <envire/core/EventHandler.hpp>
#include <envire/core/EventTypes.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <boost/lexical_cast.hpp>

namespace envire
{
    /* Generic implementation for registering new types in the serialization
     * system
     *
     * Simply define a global variable in a compilation unit, as
     *
     * <code>
     * static envire::SerializationPlugin<MLSGrid> plugin;
     * </code>
     *
     * Note that you don't need to do that explicitely if you use
     * ENVIRONMENT_ITEM_DEF
     */
    template<typename T>
    struct SerializationPlugin
    {
        SerializationPlugin () {
            SerializationFactory::addClass( T::className, &createItem<T> );
        }
        SerializationPlugin ( const std::string& className ) {
            SerializationFactory::addClass( className, &createItem<T> );
        }
    };
    
    class YAMLSerializationImpl;
    
    /**
     * Interface Class for the Serialization.
     * It provides interfaces to read/write yaml data and 
     * i/ostreams for data in binary form.
     */
    class Serialization
    {
    protected:
        YAMLSerializationImpl* yamlSerialization;
    public:
        Serialization();
        virtual ~Serialization();

        /** Prepare this serialization object to accept calls to write() and
         * read() without a backing serialization system
         */
        virtual void begin();

        /** Must be called after begin() when you are done with the
         * serialization object
         */
        virtual void end();
        
        /**
         * Writes a key and a value in the a yaml map node.
         */
        template <class T> void write(const std::string &key, const T& value);
        virtual void write(const std::string &key, const std::string &value);
        virtual void write(const std::string &key, const FrameNode::TransformType &value);

        /**
         * Reads the value at the key position from a yaml map node.
         */
        template <class T> bool read(const std::string &key, T& value);
        template <typename T> T read(const std::string& key)
        {
            T value = T();
            read(key, value);
            return value;
        }
        virtual bool read(const std::string &key, std::string &value);
        virtual bool read(const std::string &key, FrameNode::TransformType &value);

        /**
         * @return true if the key is available in the current map node
         */
        virtual bool hasKey(std::string const& key) const;
        
        /**
         * Exception thrown when getBinaryInputStream is called with stream
         * names that do not exist
         */
        class NoSuchBinaryStream : public std::runtime_error
        {
        public:
            NoSuchBinaryStream(std::string const& msg)
                : std::runtime_error(msg) {}
        };
        
        /**
         * @return an istream for a given filename
         */
        virtual std::istream& getBinaryInputStream(const std::string &filename);
        
        /**
         * @return an ostream for a given filename
         */
        virtual std::ostream& getBinaryOutputStream(const std::string &filename);
    };
    
    template <class T> bool Serialization::read(const std::string &key, T& value)
    {
        std::string tmp;
        if( read( key, tmp ) )
        {
            value = boost::lexical_cast<T>(tmp);
            return true;
        }
        return false;
    }

    template <class T> void Serialization::write(const std::string &key, const T& value)
    {
        write( key, boost::lexical_cast<std::string>(value) );
    }
    
    /**
     * The FileSerialization writes or reads the complete Environment in 
     * serialized form to or from a given directory.
     * Variables of the Items will be stored in a editable yaml-file.
     * The map representation of the items will be stored separately in
     * binary files.
     */
    class FileSerialization : public Serialization
    {
    protected:
        std::string sceneDir;
        std::vector<std::ifstream*> ifstreams;
        std::vector<std::ofstream*> ofstreams;
        
    public:
        /* name of the yaml file */
        static const std::string STRUCTURE_FILE;
        
        FileSerialization();
        ~FileSerialization();
        
        /**
         * Unserializes the Environment from a given directory.
         * @return the environment
         */
        Environment* readFromFile(const std::string &path);
        
        /**
         * Serializes a given Environment to the given directory.
         * @return true on success
         */
        bool writeToFile( Environment* env, const std::string &path );
        
        /**
         * Opens an ifstream for the given filename in the current sceneDir.
         * The streams will be stored in a vector and closed and deleted later on in 
         * the methods writeToFile or readFromFile.
         * @return an istream for a given filename
         */
        virtual std::istream& getBinaryInputStream(const std::string &filename);
        
        /**
         * Opens an ofstream for the given filename in the current sceneDir.
         * The streams will be stored in a vector and closed and deleted later on in 
         * the methods writeToFile or readFromFile.
         * @return an istream for a given filename
         */
        virtual std::ostream& getBinaryOutputStream(const std::string &filename);
        
        /**
         * Sets the path to the current serialization dir.
         */
        void setSceneDir(const std::string dir);
        
        /**
         * This is used from envire::Grid, because 
         * GDAL serialization cannot handle streams.
         * @return the serialization path
         */
        virtual const std::string getMapPath() const;
    };
    
    /**
     * The BinarySerialization stores or extracts one EnvironmentItem 
     * to or from an EnvireBinaryEvent.
     * The Variables will be stored in yaml form in a vector of bytes.
     * The map representation of the items will be stored in separate 
     * vectors of bytes.
     */
    class BinarySerialization : public Serialization
    {
    protected:
        std::map<std::string, std::stringstream*> stringstreams;
        
    public:
        BinarySerialization();
        ~BinarySerialization();
        
        /**
         * Unserializes a EnvironmentItem from a given EnvireBinaryEvent.
         * @return the EnvironmentItem
         */
        EnvironmentItem* unserializeBinaryEvent(const EnvireBinaryEvent& bin_item);
        
        /**
         * Serializes a given EnvironmentItem to a given EnvireBinaryEvent.
         * @return true on success
         */
        bool serializeBinaryEvent(EnvironmentItem* item, EnvireBinaryEvent& bin_item);
        
        /**
         * The streams, if for the given filename in the EnvireBinaryEvent 
         * available, are stored in stringstreams and will be deleted later on 
         * in the methods unserializeBinaryEvent or serializeBinaryEvent.
         * @return an istream for a given filename
         */
        virtual std::istream& getBinaryInputStream(const std::string &filename);
        
        /**
         * Creates a stringstream for the given filename. The streams will be stored 
         * in a map[filename] and will be copied to the EnvireBinaryEvent and deleted 
         * later on in the methods unserializeBinaryEvent or serializeBinaryEvent.
         * @return an ostream for a given filename
         */
        virtual std::ostream& getBinaryOutputStream(const std::string &filename);
        
    protected:
        /**
         * Deletes all entries of stringstreams.
         */
        void cleanUp();
    };
    
    
    class SynchronizationEventHandler : public EventHandler
    {
    public:
        void handle( const Event& message );
        
        virtual void handle( EnvireBinaryEvent* binary_event ) = 0;

    protected:
        BinarySerialization serialization;
    };
}
#endif
