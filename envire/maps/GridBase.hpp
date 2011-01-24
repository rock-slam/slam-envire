#ifndef __ENVIRE_GRIDBASE_HPP__
#define __ENVIRE_GRIDBASE_HPP__

#include <envire/Core.hpp>

namespace envire 
{
    class GridBase : public Map<2> 
    {
    public:
	static const std::string className;

    protected:
	size_t width, height;
	double scalex, scaley;	

    public:
	GridBase(size_t width, size_t height, double scalex, double scaley);
	~GridBase();
	GridBase(Serialization& so);
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	bool toGrid( double x, double y, size_t& m, size_t& n );
	void fromGrid( size_t m, size_t n, double& x, double& y );

	size_t getWidth() { return width; };
	size_t getHeight() { return height; };

	double getScaleX() { return scalex; };
	double getScaleY() { return scaley; };

	Extents getExtents() const;
    };
}

#endif
