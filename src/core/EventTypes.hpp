#ifndef __ENVIRE_EVENT_TYPES_HPP__
#define __ENVIRE_EVENT_TYPES_HPP__

#include <vector>
#include <string>
#include <stdint.h>

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
    struct EnvireBinaryEvent
    {
        // event part
        long id_a, id_b;
        event::Type type;
        event::Operation operation;
        
        // binary item part
        std::string className;
        std::vector<uint8_t> yamlProperties;
        std::vector<std::string> binaryStreamNames;
        std::vector< std::vector<uint8_t> > binaryStreams;
        
        EnvireBinaryEvent()
         : id_a(-1), id_b(-1), className("") {};
        EnvireBinaryEvent(event::Type type, event::Operation operation, long id_a, long id_b)
         : id_a(id_a), id_b(id_b), type(type), operation(operation), className("") {};
    };

}
#endif