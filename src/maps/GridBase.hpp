#ifndef __ENVIRE_GRIDBASE_HPP__
#define __ENVIRE_GRIDBASE_HPP__

#include <envire/core/Serialization.hpp>
#include <envire/Core.hpp>
#include <base/pose.h>
#include <boost/function.hpp>

namespace envire 
{
    /** Base class for all maps that function as regular grids
     *
     * This map offers a common interface for all maps that are regular grids
     *
     * The create() static method can be used as a factory method for all
     * subclasses that are registered on the serialization system
     *
     * A generic implementation as a template is Grid<T>, declared in Grid.hpp.
     * Specialized implementation for common types of maps can be found in
     * Grids.hpp (such as: ElevationMap, OccupancyGrid, ...)
     */
    class GridBase : public Map<2> 
    {
    public:
	static const std::string className;

        /** Internal structure used to represent a position on the grid itself,
         * as a cell index
         */
	struct Position
	{
            /** The cell position along the X axis */
	    size_t x;
            /** The cell position along the Y axis */
	    size_t y;

	    Position() {}
            Position( size_t x, size_t y ) : x(x), y(y) {}
            explicit Position( const Eigen::Vector2i &pos ) : x(pos.x()), y(pos.y()) {}
	    bool operator<( const Position& other ) const
	    {
		if( x < other.x )
		    return true;
		else
		    if( x == other.x )
			return y < other.y;
		    else
			return false;
	    }
	    bool operator==( const Position& other ) const
	    {
		return x == other.x && y == other.y;
	    }
	    bool operator!=( const Position& other ) const
	    {
		return x != other.x || y != other.y;
	    }
	};
	typedef Eigen::Vector2d Point2D;
	typedef Eigen::AlignedBox<int, 2> CellExtents;

    protected:
        
        /**
         * To Grid function that returns a grid coordinate that is multiplier 
         * higher resolved as the grid.
         * 
         * Computes grid coordinates from world coordinate.
         * */
        bool toGridTimesX(const Point2D &pWorld, Position& pGrid, int multiplier) const;
        /**
         * To Grid function that returns a grid coordinate that is multiplier 
         * higher resolved as the grid.
         * 
         * Computes grid coordinates from world coordinate.
         * */
        bool toGridTimesX(double x, double y, size_t& xi, size_t& yi, int multiplier) const;
        
	size_t cellSizeX, cellSizeY;
	double scalex, scaley;	
	double offsetx, offsety;

    public:
        typedef boost::intrusive_ptr<GridBase> Ptr;

        explicit GridBase(std::string const& id = Environment::ITEM_NOT_ATTACHED);
	/** @brief Constructor of the abstract GridBase class
	 * 
	 * Defines the extends and positioning of the grid. The grid is assumed
	 * to be on the x-y plane of the reference frame. The number of grid
	 * cells is given by the cellSizeX and cellSizeY params. Each dimension
	 * also has an scaling and offset parameter, such that the origin of the
	 * grid can be moved around and the grid scaled.
	 *
	 * The relation between the grid cell index xi and the value for the
	 * dimension x is:
	 * @verbatim
	 * x = xi * scale_x + offset_x
	 * @endverbatim
	 * This is of course the same for the y axis as well.
	 *  
	 * @param cellSizeX - number of cells in x direction
	 * @param cellSizeY - number of cells in y direction
	 * @param scalex - scaling of the x axis (size in x per cell)
	 * @param scaley - scaling of the y axis (size in y per cell)
	 * @param offsetx - x-position of the [0,0] cell
	 * @param offsety - y-position of the [0,0] cell
	 */
	GridBase(size_t cellSizeX, size_t cellSizeY,
                double scalex, double scaley,
                double offsetx = 0.0, double offsety = 0.0,
                std::string const& id = Environment::ITEM_NOT_ATTACHED);
	~GridBase();
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

        /**
         * Helper function that computes the grid coordinates of 
         * a given oriented rectangle.
         * 
         * @param multiplier This multiplier is applied to the pose before calculation the grid coordinates
         *                      This allows to calculate subcell accurate grid position.
         * 
         * Returns false if the rectangle is not inside the grid
         * */
        bool getRectPoints(const base::Pose2D &pose, double width, double height, GridBase::Position &upLeft_g, GridBase::Position &upRight_g, GridBase::Position &downLeft_g, GridBase::Position &downRight_g, int multiplier = 1) const;

        /**
         * This function calls the given callback for each cell, which 
         * will be covered by the given rectangle at the given pose.
         * 
         * Node that this method might produce some aliasing artifacts
         * at the border of the rectangle.  
         * 
         * returns true if the given rectangle is inside the grid.
         *         false otherwise.
         * */
        bool forEachInRectangle(const base::Pose2D &rectCenterWorld, double widthWorld, double heightWorld, boost::function<void (size_t, size_t)> callbackGrid) const;

        bool forEachInRectangles(const base::Pose2D &rectCenter_w, double innerWidth_w, double innerHeight_w, boost::function<void (size_t, size_t)> innerCallback, 
                                                        double outerWidth_w, double outerHeight_w, boost::function<void (size_t, size_t)> outerCallback) const;

        /** Converts coordinates from the frame specified by \c frame to the
         * map-local grid coordinates
         *
         * If the frame is not specified, the map's own frame is used (i.e. it
         * is equivalent to the other forms of toGrid)
         *
         * @return { true if (x, y) is within the grid and false otherwise. If
         * false is returned, the values of \c xi and \yi are not set }
         */
	bool toGrid(Eigen::Vector3d const& point, size_t& xi, size_t& yi, FrameNode const* frame = 0) const;

        /** Converts coordinates in the map-local frame to grid coordinates
         *
         * @return true if (x, y) is within the grid and false otherwise
         */
	bool toGrid(double x, double y, size_t& xi, size_t& yi) const;
	
        /** Converts coordinates in the map-local frame to grid coordinates
	 * and calculate the position within the cell, such that xmod and ymod
	 * are between 0 and scalex or scaley respectively
         *
         * @return true if (x, y) is within the grid and false otherwise
         */
	bool toGrid(double x, double y, size_t& xi, size_t& yi, double& xmod, double& ymod) const;

        /** Converts coordinates from the map-local grid coordinates to
         * the coordinates in the specified \c frame
         *
         * If the frame is not specified, the map's own frame is used (i.e. it
         * is equivalent to the other forms of fromGrid)
         */
        Eigen::Vector3d fromGrid(size_t xi, size_t yi, FrameNode const* frame = 0) const;

        /** Converts coordinates from the map-local grid coordinates to
         * coordinates in the map-local frame
         */
	void fromGrid(size_t xi, size_t yi, double& x, double& y) const;

	bool toGrid(const Point2D& point, Position& pos) const;
	void fromGrid(const Position& pos, Point2D& point) const;
	Point2D fromGrid(const Position& pos) const;

	bool contains( const Position& pos ) const;

        /** @deprecated
         *
         * Use getCellSizeX() instead
         */
	size_t getWidth() const { return cellSizeX; };

        /** @deprecated
         *
         * Use getCellSizeY() instead
         */
	size_t getHeight() const { return cellSizeY; };

        /** Returns the size of the grid, in cells, along the X direction
         */
        size_t getCellSizeX() const { return cellSizeX; }

        /** Returns the size of the grid, in cells, along the Y direction
         */
        size_t getCellSizeY() const { return cellSizeY; }

        /** Returns the world size of the grid along the X direction
         */
        size_t getSizeX() const { return cellSizeX * scalex; }

        /** Returns the world size of the grid along the Y direction
         */
        size_t getSizeY() const { return cellSizeY * scaley; }

        /** Returns the world size of a cell along the X direction
         */
	double getScaleX() const { return scalex; };
        /** Returns the world size of a cell along the Y direction
         */
	double getScaleY() const { return scaley; };

        /** Returns the X part of the position of the (0, 0) cell w.r.t. the
         * grid's frame
         */
	double getOffsetX() const { return offsetx; };
        /** Returns the Y part of the position of the (0, 0) cell w.r.t. the
         * grid's frame
         */
	double getOffsetY() const { return offsety; };

        /** Returns the position of the center of the grid, in world
         * coordinates, w.r.t. the position of the (0, 0) cell
         */
	Point2D getCenterPoint() const { 
	    return Point2D( cellSizeX * scalex, cellSizeY * scaley ) * 0.5
		+ Point2D( offsetx, offsety );
	};

        /** Returns the size of the grid, in world units.
	 * uses getCellExtents() as the basis
         */
	Extents getExtents() const;

	/** return the extents of the subset of the grid, which 
	 * for which the cells contain data. The base implementation is
	 * to return the cellsSize values. Override since this is used to
	 * calculate the return value for getExtents. 
	 */
	virtual CellExtents getCellExtents() const; 

        /** Read a band from a GDAL file and returns a Grid map containing the
         * loaded data
         *
         * @arg path the path to the GDAL file
         * @arg band_name the band name in the created Grid instance
         * @arg band the band index in the GDAL file
         */
        static std::pair<GridBase::Ptr, Transform> readGridFromGdal(std::string const& path, std::string const& band_name, int band = 1);

        /** Copies the specified band in this grid map
         *
         * @arg target_name the name of the new band. If omitted, uses \c band_name
         * @throw {std::runtime_error if it is not implemented for this grid and
         * std::bad_dynamic_cast if GridBase and \c this are not of the same
         * type }
         */
        virtual void copyBandFrom(GridBase const& source, std::string const& band_name, std::string const& target_name = "");

        /** Creates a new grid of the specified type and parameters
         */
        static Ptr create(std::string const& type_name,
                size_t cellSizeX, size_t cellSizeY,
                double scalex = 1, double scaley = 1,
                double offsetx = 0, double offsety = 0);

        /** @brief Checks if two grids are approximately aligned
         *
	 * It returns true if both grids are of the same dimensions, and the
	 * maximum misalignment error is less than a half cell. 
         */
        bool isAlignedWith(GridBase const& grid) const;

	/** @brief checks if two grids are approximately aligned on the cell level
	 *
	 * @return true if both grids have the same scaling and are aligned on cell 
	 * level. Grids can be of different size.
	 */
	bool isCellAlignedWith(GridBase const& grid) const;
    };
}

#endif
