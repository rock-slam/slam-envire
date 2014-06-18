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
         : time(base::Time::now()), id_a(""), id_b(""), className("") {};
        BinaryEvent(event::Type type, event::Operation operation, std::string id_a, std::string id_b)
         : time(base::Time::now()), id_a(id_a), id_b(id_b), type(type), operation(operation), className("") {};

        /** Sets values in @a this using the data in @a event, modifying @a
         * event in the process
         *
         * In C++11, we would use a move constructor
         */
        void move(BinaryEvent& other_event)
        {
            time = other_event.time;
            id_a = other_event.id_a;
            id_b = other_event.id_b;
            type = other_event.type;
            operation = other_event.operation;

            className = other_event.className;
            std::swap(yamlProperties, other_event.yamlProperties);
            std::swap(binaryStreamNames, other_event.binaryStreamNames);
            std::swap(binaryStreams, other_event.binaryStreams);
        }
    };
    typedef BinaryEvent EnvireBinaryEvent;
    typedef std::vector<BinaryEvent> BinaryEvents;
}
#endif
