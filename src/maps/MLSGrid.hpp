#ifndef __MULTILEVELSURFACEGRID_HPP__
#define __MULTILEVELSURFACEGRID_HPP__

#include <envire/maps/GridBase.hpp>
#include <boost/multi_array.hpp>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/pool.hpp>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <set>

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
     *   <li>patches registered for a certain (x, y) position can be iterated
     *       using MLSGrid::beginCell(m, n) and
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
	    SurfacePatch() {};
	    SurfacePatch( double mean, double stdev, double height = 0, double horizontal = true )
		: mean(mean), stdev(stdev), height(height), horizontal(horizontal), update_idx(0) {};

            /** Experimental code. Don't use it unless you know what you are
             * doing */
	    double distance( const SurfacePatch& other ) const
	    {
		if( !horizontal && !other.horizontal )
		    return 0;
		if( !horizontal )
		    return other.mean > mean ?
			other.mean - mean :
			std::max( 0.0, mean - height - other.mean);
		if( !other.horizontal )
		    return mean > other.mean ?
			mean - other.mean :
			std::max( 0.0, other.mean - other.height - mean);
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
                if (horizontal)
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

            /** The mean Z value. This always represents the top of the patch,
             * regardless whether the patch is horizontal or vertical
             */
	    double mean;
            /** The standard deviation in Z */
	    double stdev;
            /** For vertical patches, the height of the patch */
	    double height;
            /** Horizontal patches are just a mean and standard deviation.
             * Vertical patches also have a height, i.e. the patch is a vertical
             * block between z=(mean-height) and z
             */
	    bool horizontal;

	    size_t update_idx;
	};

    protected:
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
	MLSGrid(const MLSGrid& other);
	MLSGrid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0);
	MLSGrid(Serialization& so);
	~MLSGrid();

	MLSGrid& operator=(const MLSGrid& other);

	/** @return a shallow clone of the object, which is effectively 
	 * a map with the same properties as this, but without any content.
	 */
	MLSGrid* cloneShallow() const;

	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

        /** Clears the whole map */
	void clear();

        /** Returns the iterator on the first registered patch at \c m and \c n
         */
	iterator beginCell( size_t m, size_t n );
        /** Returns the first const iterator on the first registered patch at \c
         * m and \c n
         */
	const_iterator beginCell( size_t m, size_t n ) const;
        /** @deprecated */
	const_iterator beginCell_const( size_t m, size_t n ) const
        { return beginCell(m, n); }
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
	void insertHead( size_t m, size_t n, const SurfacePatch& value );
        /** Inserts a new surface patch at the end of the patch list at
         * the given position
         */
	void insertTail( size_t m, size_t n, const SurfacePatch& value );
        /** Removes the patch pointed-to by \c position */
	iterator erase( iterator position );

        /** Finds a surface patch at \c (position.x, position.y) that matches
         * the Z information contained in \c patch (patch is used to get mean
         * and sigma Z).
         *
         * The mean Z of the returned patch has to be within \c sigma_threshold
         * patch.sigma of sigma.mean
         */
	SurfacePatch* get( const Position& position, const SurfacePatch& patch, double sigma_threshold = 3.0 );
	/** @deprecated */
	bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev);
	void updateCell( size_t m, size_t n, double mean, double stdev );
	void updateCell( size_t m, size_t n, const SurfacePatch& patch );

	void setGapSize( double gapsize ) { this->gapSize = gapsize; }
	double getGapSize() const { return gapSize; }
	void setHorizontalPatchThickness( double thickness ) { this->thickness = thickness; }
	double getHorizontalPatchThickness() const { return thickness; }

	size_t getCellCount() const { return cellcount; }
	bool empty() const { return cellcount == 0; }

    public:
	std::pair<double, double> matchHeight( const MLSGrid& other );

	void addCell( const Position& pos );
	void initIndex();
	Index* getIndex() { return index.get(); }
	Extents getExtents() { return extents; }

    protected:
	bool mergePatch( SurfacePatch& p, const SurfacePatch& o );
	typedef boost::multi_array<SurfacePatchItem*,2> ArrayType; 
	ArrayType cells;

	double gapSize;
	double thickness;
	size_t cellcount;

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
