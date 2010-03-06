#ifndef __ENVIRE_GRID_HPP__
#define __ENVIRE_GRID_HPP__

#include "Core.hpp"
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "boost/multi_array.hpp"

#include <vector>
#include <stdexcept>

namespace envire {
    class HolderBase
    {
    public:
	virtual ~HolderBase() {};
	virtual void* getData() = 0;
	template <typename T> T& get()
	{
	    return *static_cast<T*>( getData() );
	}
    };

    template <typename T>
    class Holder : public HolderBase 
    {
	T* ptr;

    public:
	Holder()
	{
	    ptr = new T();
	};

	~Holder()
	{
	    delete ptr;
	};

	void* getData() 
	{
	    return ptr;
	}
    };

    class Grid : public CartesianMap
    {
    public:
	static const std::string ELEVATION;
	static const std::string CONFIDENCE;
	static const std::string TRAVERSABILITY;

    public:
	static const std::string className;

	Grid(size_t width, size_t height, double scalex, double scaley);
	~Grid();

	Grid(Serialization& so);
	void serialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	const std::string& getClassName() const {return className;};

	Grid* clone();

	bool toGrid( double x, double y, size_t& m, size_t& n );

	bool hasData(const std::string& type);

	template <typename T>
	    boost::multi_array<T,2>& getData(const std::string& type)
	{
	    typedef boost::multi_array<T,2> array_type;
	    if( !hasData( type ) )
	    {
		data_map[type] = new Holder<array_type>;
		data_map[type]->get<array_type>()
		    .resize( boost::extents[width][height] );

	    }

	    if( typeid(*data_map[type]) != typeid(Holder<array_type>) )
	    {
		std::cerr 
		    << "type mismatch. type should be " 
		    << typeid(data_map[type]).name() 
		    << " but is " 
		    << typeid(Holder<array_type>).name()
		    << std::endl;
		throw std::runtime_error("data type mismatch.");
	    }

	    return data_map[type]->get<array_type>();
	};


    private:
	std::map<std::string, HolderBase*> data_map;
	size_t width, height;
	double scalex, scaley;
    };
}

#endif
