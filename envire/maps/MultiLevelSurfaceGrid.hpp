#ifndef __MULTILEVELSURFACEGRID_HPP__
#define __MULTILEVELSURFACEGRID_HPP__

#include <envire/maps/GridBase.hpp>
#include <boost/multi_array.hpp>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/pool.hpp>

namespace envire
{  
    class MultiLevelSurfaceGrid : public GridBase
    {
    public:
	struct SurfacePatch
	{
	    SurfacePatch() {};
	    SurfacePatch( double mean, double stdev, double height = 0, double horizontal = true )
		: mean(mean), stdev(stdev), height(height), horizontal(horizontal) {};

	    double mean;
	    double stdev;
	    double height;
	    bool horizontal;
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
		friend std::ostream& operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator_base<T1> it );
	    friend class boost::iterator_core_access;
	    friend class MultiLevelSurfaceGrid;

	    typedef typename traits<T>::Item Data;
	    typedef typename traits<T>::pItem pData;
	    typedef typename traits<T>::Value Value;

	    Data* m_item;

	    iterator_base(Data* item) : m_item(item) {}
	public:
	    iterator_base() : m_item(NULL) {}

	    void increment() 
	    { 
		m_item = m_item->next; 
	    }
	    bool equal( iterator_base const& other ) const { return m_item == other.m_item; }
	    Value& dereference() const { return *m_item; }
	};

	typedef	iterator_base<SurfacePatchItem> iterator;
	typedef iterator_base<const SurfacePatchItem> const_iterator;

    public:
	static const std::string className;

    public:
	MultiLevelSurfaceGrid(const MultiLevelSurfaceGrid& other);
	MultiLevelSurfaceGrid(size_t width, size_t height, double scalex, double scaley);
	MultiLevelSurfaceGrid(Serialization& so);
	~MultiLevelSurfaceGrid();

	MultiLevelSurfaceGrid& operator=(const MultiLevelSurfaceGrid& other);

	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	virtual const std::string& getClassName() const {return className;};

	void clear();

	iterator beginCell( size_t m, size_t n );
	const_iterator beginCell_const( size_t m, size_t n ) const;
	iterator endCell();
	const_iterator endCell_const() const;

	void insertHead( size_t m, size_t n, const SurfacePatch& value );
	void insertTail( size_t m, size_t n, const SurfacePatch& value );
	iterator erase( iterator position );

	MultiLevelSurfaceGrid* clone() const;
	void set( EnvironmentItem* other );

	bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev);
	void updateCell( size_t m, size_t n, double mean, double stdev );
	void updateCell( size_t m, size_t n, const SurfacePatch& patch );

	void setGapSize( double gapsize ) { this->gapSize = gapsize; }
	double getGapSize() const { return gapSize; }
	void setHorizontalPatchThickness( double thickness ) { this->thickness = thickness; }
	double getHorizontalPatchThickness() const { return thickness; }

	size_t getCellCount() const { return cellcount; }

    protected:
	bool mergePatch( SurfacePatch& p, const SurfacePatch& o );
	typedef boost::multi_array<SurfacePatchItem*,2> ArrayType; 
	ArrayType cells;

	double gapSize;
	double thickness;
	size_t cellcount;

	boost::pool<> mem_pool;
    };

    template <class T>
    std::ostream& operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator_base<T> it )
    {
	os << it.m_item;
	return os;
    }
}

#endif
