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
    class Grid : public CartesianMap
    {
    public:
	static const std::string ELEVATION_MIN;
	static const std::string ELEVATION_MAX;
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

	template <class T>
	    boost::multi_array<T,2>& getGridData( const std::string& key )
	{
	    typedef boost::multi_array<T,2> array_type;
	    array_type& data( getData<array_type>(key) );
	    data.resize( boost::extents[width][height] );
	    return data;
	};

	const std::string& getClassName() const {return className;};

	Grid* clone();

	bool toGrid( double x, double y, size_t& m, size_t& n );

    private:
	size_t width, height;
	double scalex, scaley;
    };
}

#endif
