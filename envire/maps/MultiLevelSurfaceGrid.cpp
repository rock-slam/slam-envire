#include "MultiLevelSurfaceGrid.hpp"

using namespace envire;

const std::string MultiLevelSurfaceGrid::className = "envire::MultiLevelSurfaceGrid";


MultiLevelSurfaceGrid::iterator::iterator()
    : initialized( false ), item(NULL)
{
}

MultiLevelSurfaceGrid::iterator::iterator(size_t n, size_t m, SurfacePatchItem* item)
    : initialized( true ), m(m), n(n), item(item)
{
}

bool MultiLevelSurfaceGrid::iterator::isValid()
{
    return initialized && item;
}

MultiLevelSurfaceGrid::iterator& MultiLevelSurfaceGrid::iterator::operator ++()
{
    if( initialized && item )
	item = item->next;

    return *this;
}

MultiLevelSurfaceGrid::iterator MultiLevelSurfaceGrid::iterator::operator ++(int)
{
    iterator result( *this );
    ++(*this);
    return result;
}

MultiLevelSurfaceGrid::SurfacePatch& MultiLevelSurfaceGrid::iterator::operator*()
{
    if( initialized && item )
	return *item;
    else
	throw std::runtime_error("iterator not initialized");
}

MultiLevelSurfaceGrid::SurfacePatch* MultiLevelSurfaceGrid::iterator::operator->()
{
    return &*(*this);
}

bool MultiLevelSurfaceGrid::iterator::operator ==(const iterator& other) const 
{
    return( initialized == other.initialized && m == other.m && n == other.n && item == other.item );
}

std::ostream& envire::operator <<( std::ostream& os, const MultiLevelSurfaceGrid::iterator it )
{
    os << it.item;
    return os;
}

MultiLevelSurfaceGrid::MultiLevelSurfaceGrid(size_t width, size_t height, double scalex, double scaley)
    : GridBase( width, height, scalex, scaley ), cells( boost::extents[width][height] )
{
}

MultiLevelSurfaceGrid::MultiLevelSurfaceGrid(Serialization& so)
    : GridBase(so)
{
    unserialize(so);
}

MultiLevelSurfaceGrid::~MultiLevelSurfaceGrid()
{
}

void MultiLevelSurfaceGrid::serialize(Serialization& so)
{
    GridBase::serialize(so);
    so.setClassName( className );

    writeMap( getMapFileName(so.getMapPath()) + ".mls" );
}

void MultiLevelSurfaceGrid::unserialize(Serialization& so)
{
    so.setClassName( className );

    cells.resize( boost::extents[width][height] );
    readMap( getMapFileName(so.getMapPath()) + ".mls" );
}

struct SurfacePatchStore : MultiLevelSurfaceGrid::SurfacePatch
{
    SurfacePatchStore() {};

    SurfacePatchStore( const MultiLevelSurfaceGrid::SurfacePatch& data, size_t m, size_t n )
	: SurfacePatch(data), m(m), n(n) {}

    size_t m, n;
};

void MultiLevelSurfaceGrid::writeMap(const std::string& path)
{
    std::ofstream os(path.c_str());
    os << "mls" << std::endl;
    os << "1.0" << std::endl;
    os << sizeof( SurfacePatchStore ) << std::endl;
    os << "bin" << std::endl;

    for(size_t m=0;m<width;m++)
    {
	for(size_t n=0;n<height;n++)
	{
	    iterator it = beginCell( m,n );
	    while( it.isValid() )
	    {
		SurfacePatchStore d( *it, m, n );
		os.write( reinterpret_cast<const char*>(&d), sizeof( SurfacePatchStore ) );
		it++;
	    }
	}
    }

    os.close();
}

void MultiLevelSurfaceGrid::readMap(const std::string& path)
{
    std::ifstream is(path.c_str());
    
    char c[32];
    is.getline(c, 20);
    if( std::string(c) != "mls" )
	throw std::runtime_error("bad magic " + std::string(c));

    is.getline(c, 20);
    if( boost::lexical_cast<float>(std::string(c)) > 1.0 )
	throw std::runtime_error("version not supported " + std::string(c) );

    is.getline(c, 20);
    if( boost::lexical_cast<int>(std::string(c)) != sizeof( SurfacePatchStore ) )
	throw std::runtime_error("binary size mismatch");

    is.getline(c, 20);
    if( std::string(c) != "bin" )
	throw std::runtime_error("missing bin identifier" + std::string(c));

    SurfacePatchStore d;
    while( is.read(reinterpret_cast<char*>(&d), sizeof( SurfacePatchStore ) ) )
    {
	insertTail( d.m, d.n, d );
    }

    is.close();
}

MultiLevelSurfaceGrid::iterator MultiLevelSurfaceGrid::beginCell( size_t m, size_t n )
{
    return iterator( n, m, cells[m][n] );
}

MultiLevelSurfaceGrid::iterator MultiLevelSurfaceGrid::endCell( size_t m, size_t n )
{
    return iterator( n, m, NULL );
}

void MultiLevelSurfaceGrid::insertHead( size_t m, size_t n, const SurfacePatch& value )
{
    SurfacePatchItem item( value );
    item.next = cells[m][n];

    items.push_back( item );

    cells[m][n] = &items.back();
}

void MultiLevelSurfaceGrid::insertTail( size_t m, size_t n, const SurfacePatch& value )
{
    iterator last, it;
    last = it = beginCell( m, n );
    while( it.isValid() )
    {
	last = it;
	it++;
    }

    SurfacePatchItem item( value );
    item.next = NULL;

    items.push_back( item );

    if( last.isValid() )
	last.item->next = &items.back();
    else
	cells[m][n] = &items.back();
}

MultiLevelSurfaceGrid* MultiLevelSurfaceGrid::clone()
{
    return new MultiLevelSurfaceGrid(*this);
}

bool MultiLevelSurfaceGrid::get(const Eigen::Vector3d& position, double& zpos, double& zstdev)
{
    size_t x, y;
    if( toGrid(position.x(), position.y(), x, y) )
    {
	MultiLevelSurfaceGrid::iterator it = beginCell(x,y);
	while( it.isValid() )
	{
	    MultiLevelSurfaceGrid::SurfacePatch &p(*it);
	    const double interval = (zstdev + p.stdev) * 3.0;
	    if( position.z() - interval < p.mean && 
		    position.z() + interval > p.mean )
	    {
		zpos = p.mean;
		zstdev = p.stdev;
		return true;
	    }
	    it++;
	}
    }
    return false;
}
