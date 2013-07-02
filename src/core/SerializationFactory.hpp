#ifndef __ENVIRE_SERIALIZATION_FACTORY_HPP__
#define __ENVIRE_SERIALIZATION_FACTORY_HPP__

#include <map>
#include <string>
// #include <envire/core/Serialization.hpp>

namespace envire
{
    class EnvironmentItem;
    class Serialization;

    class SerializationFactory 
    {
    public:
        /** Factory typedef, which needs to be implemented by all EnvironmentItems
         * that are serialized.
         */
        typedef EnvironmentItem* (*Factory)(Serialization &);

        /** Stores the mapping for all classes that can be serialized, and a function
         * pointer to the Factory method of that class.
         */
        static std::map<std::string, Factory>& getMap();

        /** create and object for the given class. Will throw if no such class is registered.
         */
        static EnvironmentItem* createObject( const std::string& className, Serialization& so );

        /** register a class with the factory
         */
        static void addClass( const std::string& className, Factory f );
    };
}
    
#endif