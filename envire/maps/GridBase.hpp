#ifndef __ENVIRE_GRIDBASE_HPP__
#define __ENVIRE_GRIDBASE_HPP__

#include <envire/Core.hpp>

namespace envire 
{
    class GridBase : public Map<2> 
    {
    public:
	static const std::string className;

	struct Position 
	{
	    size_t m;
	    size_t n;

	    Position( size_t m, size_t n ) : m(m), n(n) {}
	    bool operator<( const Position& other ) const
	    {
		if( m < other.m )
		    return true;
		else
		    if( m == other.m )
			return n < other.n;
		    else
			return false;
	    }
	};

    protected:
	size_t width, height;
	double scalex, scaley;	

    public:
	GridBase(size_t width, size_t height, double scalex, double scaley);
	~GridBase();
	GridBase(Serialization& so);
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	bool toGrid( double x, double y, size_t& m, size_t& n ) const;
	void fromGrid( size_t m, size_t n, double& x, double& y ) const;

	size_t getWidth() const { return width; };
	size_t getHeight() const { return height; };

	double getScaleX() const { return scalex; };
	double getScaleY() const { return scaley; };

	Extents getExtents() const;
    };
}

#endif
