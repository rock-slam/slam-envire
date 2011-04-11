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

	    Position() {}
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
	typedef Eigen::Vector2d Point2D;

    protected:
	size_t width, height;
	double scalex, scaley;	

    public:
	GridBase(size_t width, size_t height, double scalex, double scaley);
	~GridBase();
	GridBase(Serialization& so);
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

        /** Converts coordinates from the frame specified by \c frame to the
         * map-local grid coordinates
         *
         * If the frame is not specified, the map's own frame is used (i.e. it
         * is equivalent to the other forms of toGrid)
         *
         * @return { true if (x, y) is within the grid and false otherwise. If
         * false is returned, the values of \c m and \n are not set }
         */
	bool toGrid(Eigen::Vector3d const& point, size_t& m, size_t& n, FrameNode const* frame = 0) const;

        /** Converts coordinates in the map-local frame to grid coordinates
         *
         * @return true if (x, y) is within the grid and false otherwise
         */
	bool toGrid(double x, double y, size_t& m, size_t& n) const;

        /** Converts coordinates from the map-local grid coordinates to
         * the coordinates in the specified \c frame
         *
         * If the frame is not specified, the map's own frame is used (i.e. it
         * is equivalent to the other forms of fromGrid)
         */
        Eigen::Vector3d fromGrid(size_t m, size_t n, FrameNode const* frame = 0) const;

        /** Converts coordinates from the map-local grid coordinates to
         * coordinates in the map-local frame
         */
	void fromGrid(size_t m, size_t n, double& x, double& y) const;

	bool toGrid(const Point2D& point, Position& pos) const;
	void fromGrid(const Position& pos, Point2D& point) const;
	Point2D fromGrid(const Position& pos) const;

	bool contains( const Position& pos ) const;

	size_t getWidth() const { return width; };
	size_t getHeight() const { return height; };

	double getScaleX() const { return scalex; };
	double getScaleY() const { return scaley; };

	Point2D getCenterPoint() const { return Point2D( width * scalex, height * scaley ) * 0.5; };

	Extents getExtents() const;
    };
}

#endif
