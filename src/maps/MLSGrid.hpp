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

#include <envire/maps/MLSPatch.hpp>

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
