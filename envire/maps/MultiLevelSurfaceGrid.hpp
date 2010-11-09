#ifndef __MULTILEVELSURFACEGRID_HPP__
#define __MULTILEVELSURFACEGRID_HPP__

#include <envire/maps/GridBase.hpp>
#include <boost/multi_array.hpp>

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
	    SurfacePatchItem( const SurfacePatch& data ) : SurfacePatch( data ) {};

	    SurfacePatchItem* next;
	};

	class iterator
	{
	public:
	    friend std::ostream& envire::operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator it );
	    friend class MultiLevelSurfaceGrid;

	protected:
	    bool initialized;

	    size_t m, n;
	    SurfacePatchItem* item;

	public:
	    iterator();
	    iterator(size_t n, size_t m, SurfacePatchItem* item);

	    bool isValid();

	    iterator& operator ++();
	    iterator operator ++(int unused);

	    bool operator ==(const iterator& other) const;

	    SurfacePatch& operator*();
	    SurfacePatch* operator->();
	};

    public:
	static const std::string className;

    public:
	MultiLevelSurfaceGrid(size_t width, size_t height, double scalex, double scaley);
	MultiLevelSurfaceGrid(Serialization& so);
	~MultiLevelSurfaceGrid();

	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	virtual const std::string& getClassName() const {return className;};

	iterator beginCell( size_t m, size_t n );
	iterator endCell( size_t m, size_t n );

	void insertHead( size_t m, size_t n, const SurfacePatch& value );
	void insertTail( size_t m, size_t n, const SurfacePatch& value );

	MultiLevelSurfaceGrid* clone();

	bool get(const Eigen::Vector3d& position, double& zpos, double& zstdev);

    protected:
	typedef boost::multi_array<SurfacePatchItem*,2> ArrayType; 
	ArrayType cells;

	std::list<SurfacePatchItem> items;
    };

    std::ostream& operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator it );
}

#endif
