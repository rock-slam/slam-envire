#ifndef __MULTILEVELSURFACEGRID_HPP__
#define __MULTILEVELSURFACEGRID_HPP__

#include <envire/maps/GridBase.hpp>
#include <boost/multi_array.hpp>
#include <envire/core/Serialization.hpp>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/pool.hpp>
#include <boost/shared_ptr.hpp>

#include <base/geometry/spline.h>

#include <algorithm>
#include <set>

#include <base/eigen.h>

#include <envire/maps/MLSPatch.hpp>
#include <envire/maps/MLSConfiguration.hpp>
#include <envire/tools/ListGrid.hpp>

namespace envire
{  
    /** Representation of a multi-level surface grid
     *
     * This class represents grids of cells in which each cell hold a
     * multi-level information (in the form of a
     * MLSGrid::SurfacePatch data structure)
     *
     * Patches can be accessed by two methods:
     * <ul>
     *   <li>a patch matching a certain (x, y, z, sigma_z) position can be
     *       returned by MLSGrid::get</li>
     *   <li>patches registered for a certain (xi, yi) cell index can be iterated
     *       using MLSGrid::beginCell(xi, yi) and
     *       MLSGrid::endCell(). Both a MLSGrid::const_iterator and
     *       MLSGrid::iterator iterators are provided for that purpose
     * </ul>
     *
     * Merged sets of MLSGrid instances can be managed with the
     * MLSMap map class.
     */
    class MLSGrid : public GridBase
    {
	ENVIRONMENT_ITEM( MLSGrid )

    public:
	typedef envire::SurfacePatch SurfacePatch;
	typedef envire::MLSConfiguration Configuration;

	/** 
	 * index class stores a list of cell positions that are occupied in the
	 * grid.  By default the index in the mls is switched off. You have to
	 * call initIndex on the mls to activate.
	 */
	struct Index 
	{
	    std::set<Position> cells;
	    void addCell( const Position& pos )
	    {
		cells.insert( pos );
	    }

	    void reset() { cells.clear(); }
	};

    protected:
	ListGrid<SurfacePatch> cells;

    public:
	typedef	ListGrid<SurfacePatch>::iterator iterator;
	typedef ListGrid<SurfacePatch>::const_iterator const_iterator;

        /**
         * Creates the grid with the specified parameters.\n
         * width,height: Number of horizontal and vertical patches.\n
         * scalex, scaley: Size of each patch.\n
         * offsetx, offsety: Describing the world_to_mls transformation.
         */
        MLSGrid();
	MLSGrid(const MLSGrid& other);
	MLSGrid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0);
	~MLSGrid();

	MLSGrid& operator=(const MLSGrid& other);

	/** @return a shallow clone of the object, which is effectively 
	 * a map with the same properties as this, but without any content.
	 */
	MLSGrid* cloneShallow() const;

	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	void writeMap(std::ostream& os);
	void readMap(std::istream& is);

        /** Clears the whole map */
	void clear();

	/**
	 * This function expects a spline in world coordinates that
	 * get's projected on top of the surface of the mls grid.
	 * */
	base::geometry::Spline3 projectSplineOnSurface(double startHeight, const base::geometry::Spline3 &spline, const double zOffset = 0.0);

	/**
	 * This function expects an array of local 
	 * grid positions and returns an array
	 * of local grid coordinates with Z positions 
	 * on top of surface of the mls grid. 
	 * */
	std::vector<Eigen::Vector3d> projectPointsOnSurface(double startHeight, const std::vector<Position> &gridPoints, const double zOffset = 0.0);
	
        /** Returns the iterator on the first registered patch at \c xi and \c
         * yi
         */
	iterator beginCell( size_t xi, size_t yi );
        /** Returns the first const iterator on the first registered patch at \c
         * xi and \c yi
         */
	const_iterator beginCell( size_t xi, size_t yi ) const;
        /** Returns the past-the-end iterator for cell iteration */
	iterator endCell();
        /** Returns the const past-the-end iterator for cell iteration */
	const_iterator endCell() const;

        /** Inserts a new surface patch at the beginning of the patch list at
         * the given position
         */
	void insertHead( size_t xi, size_t yi, const SurfacePatch& value );
        /** Inserts a new surface patch at the end of the patch list at
         * the given position
         */
	void insertTail( size_t xi, size_t yi, const SurfacePatch& value );
        /** Removes the patch pointed-to by \c position */
	iterator erase( iterator position );

        /** Finds a surface patch at \c (position.x, position.y) that matches
         * the Z information contained in \c patch (patch is used to get mean
         * and sigma Z).
         *
         * The mean Z of the returned patch has to be within \c sigma_threshold
         * patch.sigma of sigma.mean
         */
	SurfacePatch* get( const Position& position, const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true );
	SurfacePatch* get( const Eigen::Vector2d& position, double& zpos, double& zstdev );
	/** 
	 * used for backwards compatibility
	 */
	SurfacePatch* get( const Eigen::Vector3d& position, double& zpos, double& zstdev );
	void updateCell( size_t xi, size_t yi, double mean, double stdev );
	void updateCell( size_t xi, size_t yi, const SurfacePatch& patch );
	void updateCell( const Position& pos, const SurfacePatch& patch );

	bool update( const Eigen::Vector2d& pos, const SurfacePatch& patch );

	size_t getCellCount() const { return cellcount; }
	bool empty() const { return cellcount == 0; }

	Configuration& getConfig() { return config; }
	const Configuration& getConfig() const { return config; }

	/** @deprecated use getConfig()...() instead */
	void setGapSize( double gapsize ) { config.gapSize = gapsize; }
	/** @deprecated use getConfig()...() instead */
	double getGapSize() const { return config.gapSize; }
	/** @deprecated use getConfig()...() instead */
	void setHorizontalPatchThickness( double thickness ) { config.thickness = thickness; }
	/** @deprecated use getConfig()...() instead */
	double getHorizontalPatchThickness() const { return config.thickness; }
	/** @deprecated use getConfig()...() instead */
	void setHasCellColor( bool use ) { config.useColor = use; }
	/** @deprecated use getConfig()...() instead */
	bool getHasCellColor() const { return config.useColor; }

    public:
	/** @deprecated
	 */
	std::pair<double, double> matchHeight( const MLSGrid& other );

	/** 
	 * merge another MLSGrid into this grid applying a transform
	 * if necessary.
	 *
	 * @param other grid to merge into this
	 * @param other2this transformation from other grid to this grid
	 * @param offset mean, stdev well be added to the other cells before
	 *        merging. Also update_idx will be used from offset
	 */
	void merge( const MLSGrid& other, const Eigen::Affine3d& other2this, const SurfacePatch& offset );

	/** 
	 * see how well the other MLSGrid matches into this one 
	 *
	 * effectively this function calculates the ratio of patches which
	 * match for cells that have at least one patch.
	 *
	 * @param other grid to match 
	 * @param other2this transformation from other grid to this grid
	 * @param offset mean, stdev well be added to the other cells before
	 *        merging. Also update_idx will be used from offset
	 * @param sampling only take a subset of 1/sampling cells to match
	 * @param sigma value to use for the matching
	 */
	float match( const MLSGrid& other, const Eigen::Affine3d& other2this, const SurfacePatch& offset, size_t sampling, float sigma );

	/** mark a cell of the grid as being used. Adds it to the index if
	 * available and updates the extents of the grid.
	 */
	void addCell( const Position& pos );

	/** after this function, an index is associated with the grid,
	 * which stores which grid cells are being used.
	 */
	void initIndex();

	/** returns a pointer to the index. The pointer is only valid
	 * if the index has been initialized through initIndex()
	 */
	const Index* getIndex() const { return index.get(); }

	/** return the extents of the subset of the grid, which 
	 * contains cells.
	 */
	CellExtents getCellExtents() const { return extents.isEmpty() ? CellExtents(Eigen::Vector2i(0,0),Eigen::Vector2i(0,0)) : extents; }

	/**
         * Moves the content of the MLSGrid by 
         * x and y cells. Cells leaving the 
         * grid will be discarded. Cells entering
         * the grid will be initialized with zero
         * */
	void move(int x, int y);
    protected:
	bool mergePatch( SurfacePatch& p, SurfacePatch& o );

	/// configuration of the mls
	Configuration config;

	/// holds the number of patches in the grid 
	size_t cellcount;

	/// optionaly stores information on which grid cells are used
	boost::shared_ptr<Index> index;
	CellExtents extents;
    };

    /** For backward compatibility. Use MLSGrid instead. */
    typedef MLSGrid MultiLevelSurfaceGrid;
}

#endif
