#include "Core.hpp"
#include "LaserScan.hpp"

#include <boost/assign/list_of.hpp>
#include "boost/filesystem.hpp"

extern "C" {
#include <yaml.h>
}

using namespace std;
using namespace envire;
using namespace boost::assign;
using namespace boost::filesystem;

namespace envire 
{
    class SerializationImpl
    {
    public:
	yaml_parser_t parser;
	yaml_event_t event;
    };
}


const std::string Serialization::STRUCTURE_FILE = "scene.yml";

template<class T> EnvironmentItem* create(Serialization &so) 
{
    return new T(so);
}

map<std::string, Serialization::Factory> Serialization::classMap 
    = map_list_of("envire::FrameNode", &create<FrameNode> );


Serialization::Serialization()
{
    impl = new SerializationImpl();
}

Serialization::~Serialization()
{
    delete impl;
}

void Serialization::write(std::string& key, std::string& value)
{
}

void Serialization::write(std::string& key, long value)
{
}

std::string Serialization::readString(std::string& key)
{
}

long Serialization::readLong(std::string& key)
{
}

void Serialization::serialize(Environment *env, std::string &path_str) 
{
        
}

Environment* Serialization::unserialize(std::string &path_str) 
{
    path path( path_str ); 

}

