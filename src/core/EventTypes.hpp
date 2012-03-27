#ifndef __ENVIRE_EVENT_TYPES_HPP__
#define __ENVIRE_EVENT_TYPES_HPP__

#include <vector>
#include <string>
#include <stdint.h>
#include <base/time.h>

namespace envire
{
    namespace event
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
    }
    
    /**
     * Environment binary item, holds the data of an EnvironmentItem
     * in binary form. It is used to serialize one single EnvironmentItem.
     */
    struct BinaryEvent
    {
	// timestamp of the event
	base::Time time;

        // event part
        std::string id_a, id_b;
        event::Type type;
        event::Operation operation;
        
        // binary item part
        std::string className;
        std::vector<uint8_t> yamlProperties;
        std::vector<std::string> binaryStreamNames;
        std::vector< std::vector<uint8_t> > binaryStreams;
        
        BinaryEvent()
         : id_a(""), id_b(""), className("") {};
        BinaryEvent(event::Type type, event::Operation operation, std::string id_a, std::string id_b)
         : id_a(id_a), id_b(id_b), type(type), operation(operation), className("") {};
    };

    typedef BinaryEvent EnvireBinaryEvent;
}
#endif
