#ifndef __MULTILEVELSURFACEGRID_HPP__
#define __MULTILEVELSURFACEGRID_HPP__

#include <envire/maps/GridBase.hpp>
#include <boost/multi_array.hpp>
#include <envire/core/Serialization.hpp>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/pool.hpp>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <set>

#include <base/eigen.h>

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
	struct Index 
	{
	    std::set<Position> cells;
	    void addCell( const Position& pos )
	    {
		cells.insert( pos );
	    }

	    void reset() { cells.clear(); }
	};

        /** The representation of one surface in a cell
         *
         * This data structure either represents a surface or a vertical block,
         * depending on the value of the \c horizontal field.
         *
         * If \c horizontal is true, a surface is represented as a mean altitude
         * and standard deviation around that mean.
         *
         * If \c horizontal is false, a vertical block is represented, in which
         * the mean value is the top of the block, and mean - height the bottom
         * of it. The standard deviation still applies.
         */
	struct SurfacePatch
	{
	    friend class MLSGrid;

	    /** a surface patch can be three different types,
	     * each changing how the cell values are interpreded
	     *
	     * HORIZONTAL - is a horizontal patch, which does not have a height value
	     * VERTICAL - used to represent walls and other vertical structures, has a height
	     * NEGATIVE - represent absence of structure, otherwise equal to VERTICAL
	     */
	    enum TYPE
	    {
		VERTICAL = 0,
		HORIZONTAL = 1,
		NEGATIVE = 2
	    };

	    SurfacePatch() {};
	    SurfacePatch( float mean, float stdev, float height = 0, TYPE type = HORIZONTAL )
		: mean(mean), stdev(stdev), height(height), type(type), update_idx(0) {};

            /** Experimental code. Don't use it unless you know what you are
             * doing */
	    float distance( const SurfacePatch& other ) const
	    {
		if( !isHorizontal() && !other.isHorizontal() )
		    return 0;
		if( !isHorizontal() )
		    return other.mean > mean ?
			other.mean - mean :
			std::max( 0.0f, mean - height - other.mean);
		if( !other.isHorizontal() )
		    return mean > other.mean ?
			mean - other.mean :
			std::max( 0.0f, other.mean - other.height - mean);
		return std::abs( mean - other.mean );
	    };

            /** Returns true if the mean of \c self is lower than the mean of \c
             * other
             */
	    bool operator<( const SurfacePatch& other ) const
	    {
		return mean < other.mean;
	    };

            /** The minimum Z extent of this patch
             *
             * The uncertainty is handled by removing sigma_threshold * stdev to
             * the mean minimum Z
             */
            double getMinZ(double sigma_threshold = 2.0) const
            {
                if (isHorizontal())
                    return mean - stdev *  sigma_threshold;
                else
                    return mean - height - stdev * sigma_threshold;
            }

            /** The maximum Z extent of this patch
             *
             * The uncertainty is handled by adding sigma_threshold * stdev to
             * the mean maximum Z
             */
            double getMaxZ(double sigma_threshold = 2.0) const
            {
                return mean + stdev * sigma_threshold;
            }

	    /** flag the patch as horizontal
	     */
	    void setHorizontal()
	    {
		type = HORIZONTAL;
	    }

	    /** flag the patch as vertical
	     */
	    void setVertical()
	    {
		type = VERTICAL;
	    }

	    void setNegative()
	    {
		type = NEGATIVE;
	    }

	    /** @return true if the patch is horizontal
	     */
	    bool isHorizontal() const
	    {
		return type == HORIZONTAL;
	    }

	    /** @return true if patch is vertical
	     */
	    bool isVertical() const
	    {
		return type == VERTICAL;
	    }

	    bool isNegative() const
	    {
		return type == NEGATIVE;
	    }

	    base::Vector3d getColor() const
	    {
		return base::Vector3d( color[0], color[1], color[2] ) / 255.0;
	    }

	    void setColor( const base::Vector3d& c )
	    {
		color[0] = c[0] * 255;
		color[1] = c[1] * 255;
		color[2] = c[2] * 255;
	    }

            /** The mean Z value. This always represents the top of the patch,
             * regardless whether the patch is horizontal or vertical
             */
	    float mean;
            /** The standard deviation in Z */
	    float stdev;
            /** For vertical patches, the height of the patch */
	    float height;
            /** Horizontal patches are just a mean and standard deviation.
             * Vertical patches also have a height, i.e. the patch is a vertical
             * block between z=(mean-height) and z
             */

	protected:
	    TYPE type;

	public:
	    size_t update_idx;
	    uint8_t color[3];
	};

    protected:
	/** For each of the grid cells, there is a list of items.
	 * These items are organised as a double linked list
	 */
	struct SurfacePatchItem : public SurfacePatch
	{
	    typedef SurfacePatchItem Item;
	    typedef SurfacePatch Value;

	    SurfacePatchItem() {};
	    explicit SurfacePatchItem( const SurfacePatch& data ) : SurfacePatch( data ) {};

	    SurfacePatchItem* next;
	    SurfacePatchItem** pthis;
	};

	template <class T>
	struct traits 
	{
	    typedef typename T::Item Item;
	    typedef typename T::Value Value;
	    typedef T* pItem;
	};

	template <class T>
	struct traits<const T>
	{
	    typedef const typename T::Item Item;
	    typedef const typename T::Value Value;
	    typedef const typename T::Item* const pItem;
	};

    public:
	template <class T>
	class iterator_base :
	    public boost::iterator_facade<
		iterator_base<T>,
		typename traits<T>::Value,
		boost::forward_traversal_tag
		>
	{
	    template <class T1>
		friend std::ostream& operator <<( std::ostream& os, const MLSGrid::iterator_base<T1> it );
            template <typename> friend class iterator_base;

	    friend class boost::iterator_core_access;
	    friend class MLSGrid;

	    typedef typename traits<T>::Item Data;
	    typedef typename traits<T>::pItem pData;
	    typedef typename traits<T>::Value Value;

	    Data* m_item;

	    iterator_base(Data* item) : m_item(item) {}

	    void increment() 
	    { 
		m_item = m_item->next; 
	    }
            template<typename OtherT>
	    bool equal( iterator_base<OtherT> const& other ) const { return m_item == other.m_item; }
	    Value& dereference() const { return *m_item; }

	public:
	    iterator_base() : m_item(NULL) {}

            template<typename OtherValue>
            iterator_base(iterator_base<OtherValue> const& other)
                : m_item(other.m_item) {}
	};

	typedef	iterator_base<SurfacePatchItem> iterator;
	typedef iterator_base<const SurfacePatchItem> const_iterator;

    public:
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

        /** Returns the iterator on the first registered patch at \c xi and \c
         * yi
         */
	iterator beginCell( size_t xi, size_t yi );
        /** Returns the first const iterator on the first registered patch at \c
         * xi and \c yi
         */
	const_iterator beginCell( size_t xi, size_t yi ) const;
        /** @deprecated */
	const_iterator beginCell_const( size_t xi, size_t yi ) const
        { return beginCell(xi, yi); }
        /** Returns the past-the-end iterator for cell iteration */
	iterator endCell();
        /** Returns the const past-the-end iterator for cell iteration */
	const_iterator endCell() const;
        /** @deprecated */
	const_iterator endCell_const() const
        { return endCell(); }

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
	/** @deprecated */
	bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev);
	void updateCell( size_t xi, size_t yi, double mean, double stdev );
	void updateCell( size_t xi, size_t yi, const SurfacePatch& patch );
	void updateCell( const Position& pos, const SurfacePatch& patch );

	void setGapSize( double gapsize ) { this->gapSize = gapsize; }
	double getGapSize() const { return gapSize; }
	void setHorizontalPatchThickness( double thickness ) { this->thickness = thickness; }
	double getHorizontalPatchThickness() const { return thickness; }

	size_t getCellCount() const { return cellcount; }
	bool empty() const { return cellcount == 0; }

	void setHasCellColor( bool use ) { hasCellColor_ = use; }
	bool getHasCellColor() const { return hasCellColor_; }

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
	Extents getCellExtents() const { return extents; }

    protected:
	bool mergePatch( SurfacePatch& p, SurfacePatch& o );
	typedef boost::multi_array<SurfacePatchItem*,2> ArrayType; 
	ArrayType cells;

	double gapSize;
	double thickness;
	size_t cellcount;
	bool hasCellColor_;

	/// optionaly stores information on which grid cells are used
	boost::shared_ptr<Index> index;
	Extents extents;
	boost::pool<> mem_pool;
    };

    template <class T>
    std::ostream& operator <<( std::ostream& os, const MLSGrid::iterator_base<T> it )
    {
	os << it.m_item;
	return os;
    }

    /** For backward compatibility. Use MLSGrid instead. */
    typedef MLSGrid MultiLevelSurfaceGrid;
}

#endif
