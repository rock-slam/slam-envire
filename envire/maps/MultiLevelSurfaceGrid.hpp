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
	    SurfacePatch( double mean, double stdev, double height, double horizontal )
		: mean(mean), stdev(stdev), height(height), horizontal(horizontal) {};

	    double mean;
	    double stdev;
	    double height;
	    bool horizontal;
	};

	struct SurfacePatchItem : public SurfacePatch
	{
	    SurfacePatchItem() {};
	    explicit SurfacePatchItem( const SurfacePatch& data ) : SurfacePatch( data ) {};

	    SurfacePatchItem* next;
	};

	template <class Value, class Data>
	class iterator_base :
	    public boost::iterator_facade<
		iterator_base<Value, Data>,
		Value,
		boost::forward_traversal_tag
		>
	{
	    template <class T, class T2>
		friend std::ostream& operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator_base<T, T2> it );
	    friend class boost::iterator_core_access;
	    friend class MultiLevelSurfaceGrid;
	    Data* m_item;

	public:
	    iterator_base() : m_item(NULL) {}
	    iterator_base(Data* item) : m_item(item) {}

	    void increment() { m_item =	m_item->next; }
	    bool equal( iterator_base const& other ) const { return m_item == other.m_item; }
	    Value& dereference() const { return *m_item; }
	};

	typedef	iterator_base<SurfacePatch, SurfacePatchItem> iterator;
	typedef iterator_base<const SurfacePatch, const SurfacePatchItem> const_iterator;

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

	MultiLevelSurfaceGrid* clone() const;
	void set( EnvironmentItem* other );

	bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev);

    protected:
	typedef boost::multi_array<SurfacePatchItem*,2> ArrayType; 
	ArrayType cells;

	boost::pool<> mem_pool;
    };

    template <class T, class T2>
    std::ostream& operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator_base<T, T2> it )
    {
	os << it.m_item;
	return os;
    }
}

#endif
