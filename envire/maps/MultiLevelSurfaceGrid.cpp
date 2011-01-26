#include "MultiLevelSurfaceGrid.hpp"
#include <fstream>

using namespace envire;

const std::string MultiLevelSurfaceGrid::className = "envire::MultiLevelSurfaceGrid";

MultiLevelSurfaceGrid::MultiLevelSurfaceGrid(size_t width, size_t height, double scalex, double scaley)
    : GridBase( width, height, scalex, scaley ), cells( boost::extents[width][height] ), 
     gapSize( 1.0 ), thickness( 0.05 ), cellcount( 0 ), mem_pool( sizeof( SurfacePatchItem ) )
{
}

void MultiLevelSurfaceGrid::clear()
{
    for(size_t m=0;m<width;m++)
    {
	for(size_t n=0;n<height;n++)
	{
	    cells[m][n] = NULL;
	}
    }
    mem_pool.purge_memory();
    cellcount = 0;
}

MultiLevelSurfaceGrid::MultiLevelSurfaceGrid(const MultiLevelSurfaceGrid& other)
    : GridBase( other ), cells( boost::extents[other.width][other.height] ), mem_pool( sizeof( SurfacePatchItem ) )
{
    for(size_t m=0;m<width;m++)
    {
	for(size_t n=0;n<height;n++)
	{
	    for( const_iterator it = other.beginCell_const( m,n ); it != endCell_const(); it++ )
		insertTail( m, n, *it );
	}
    }
}

MultiLevelSurfaceGrid& MultiLevelSurfaceGrid::operator=(const MultiLevelSurfaceGrid& other)
{
    if( this != &other )
    {
	GridBase::operator=(other);

	mem_pool.purge_memory();
	cells.resize( boost::extents[width][height] );

	for(size_t m=0;m<width;m++)
	{
	    for(size_t n=0;n<height;n++)
	    {
		cells[m][n] = NULL;
		for( const_iterator it = other.beginCell_const( m,n ); it != endCell_const(); it++ )
		    insertTail( m, n, *it );
	    }
	}
    }

    return *this;
}

MultiLevelSurfaceGrid::MultiLevelSurfaceGrid(Serialization& so)
    : GridBase(so), mem_pool( sizeof( SurfacePatchItem ) )
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
	    for( iterator it = beginCell( m,n ); it != endCell(); it++ )
	    {
		SurfacePatchStore d( *it, m, n );
		os.write( reinterpret_cast<const char*>(&d), sizeof( SurfacePatchStore ) );
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
    return iterator( cells[m][n] );
}

MultiLevelSurfaceGrid::const_iterator MultiLevelSurfaceGrid::beginCell_const( size_t m, size_t n ) const
{
    return const_iterator( cells[m][n] );
}

MultiLevelSurfaceGrid::iterator MultiLevelSurfaceGrid::endCell()
{
    return iterator();
}

MultiLevelSurfaceGrid::const_iterator MultiLevelSurfaceGrid::endCell_const() const
{
    return const_iterator();
}

void MultiLevelSurfaceGrid::insertHead( size_t m, size_t n, const SurfacePatch& value )
{
    SurfacePatchItem* n_item = static_cast<SurfacePatchItem*>(mem_pool.malloc());
    static_cast<SurfacePatch&>(*n_item).operator=(value);
    n_item->next = cells[m][n];
    n_item->pthis = &cells[m][n];

    cells[m][n] = n_item;
    cellcount++;
}

void MultiLevelSurfaceGrid::insertTail( size_t m, size_t n, const SurfacePatch& value )
{
    iterator last, it;
    last = it = beginCell( m, n );
    while( it != endCell() )
    {
	last = it;
	it++;
    }

    SurfacePatchItem* n_item = static_cast<SurfacePatchItem*>(mem_pool.malloc());
    static_cast<SurfacePatch&>(*n_item).operator=(value);
    n_item->next = NULL;
    
    if( last != endCell() )
    {
	last.m_item->next = n_item;
	n_item->pthis = &last.m_item->next;
    }
    else
    {
	cells[m][n] = n_item;
	n_item->pthis = &cells[m][n];
    }

    cellcount++;
}

MultiLevelSurfaceGrid::iterator MultiLevelSurfaceGrid::erase( iterator position )
{
    SurfacePatchItem* &p( position.m_item );
    iterator res( p->next );

    *p->pthis = p->next;
    if( p->next )
	p->next->pthis = p->pthis; 

    cellcount--;

    return res; 
}

MultiLevelSurfaceGrid* MultiLevelSurfaceGrid::clone() const
{
    return new MultiLevelSurfaceGrid(*this);
}

void MultiLevelSurfaceGrid::set( EnvironmentItem* other )
{
    MultiLevelSurfaceGrid *p = dynamic_cast<MultiLevelSurfaceGrid*>( other );
    if( p ) operator=( *p );
}

bool MultiLevelSurfaceGrid::get(const Eigen::Vector3d& position, double& zpos, double& zstdev)
{
    size_t x, y;
    if( toGrid(position.x(), position.y(), x, y) )
    {
	MultiLevelSurfaceGrid::iterator it = beginCell(x,y);
	while( it != endCell() )
	{
	    MultiLevelSurfaceGrid::SurfacePatch &p(*it);
	    // TODO remove this hack
	    const double interval = (zstdev + p.stdev) * 3.0 + 0.5;
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

void MultiLevelSurfaceGrid::updateCell( size_t m, size_t n, double mean, double stdev )
{
    updateCell( m, n, SurfacePatch( mean, stdev ) );
}

void MultiLevelSurfaceGrid::updateCell( size_t m, size_t n, const SurfacePatch& o )
{
    typedef std::list<MultiLevelSurfaceGrid::iterator> iterator_list;
    iterator_list merged;

    for(MultiLevelSurfaceGrid::iterator it = beginCell( m, n ); it != endCell(); it++ )
    {
	// merge the patches and remember the ones which where merged 
	if( mergePatch( *it, o ) )
	    merged.push_back( it );
    }

    if( merged.empty() )
    {
	// insert the patch since we didn't merge it with any other
	insertHead( m, n, o );
    }
    else
    {
	// if there is more than one affected patch, merge them until 
	// there is only one left
	while( !merged.empty() )
	{
	    iterator_list::iterator it = ++merged.begin();
	    while( it != merged.end() ) 
	    {
		if( mergePatch( **merged.begin(), **it ) )
		{
		    erase( *it );
		    it = merged.erase( it );
		}
		else
		    it++;
	    }
	    merged.pop_front();
	}
    }
}

bool MultiLevelSurfaceGrid::mergePatch( SurfacePatch& p, const SurfacePatch& o )
{
    const double delta_dev = sqrt( p.stdev * p.stdev + o.stdev * o.stdev );

    // see if the distance between the patches is small enough
    if( (p.mean - p.height - gapSize - delta_dev) < o.mean 
	    && (p.mean + gapSize + delta_dev) > (o.mean - o.height) )
    {
	// if both patches are horizontal, we see if we can merge them
	if( p.horizontal && o.horizontal ) 
	{
	    if( (p.mean - p.height - thickness - delta_dev) < o.mean && 
		    (p.mean + thickness + delta_dev) > o.mean )
	    {
		// for horizontal patches, perform an update similar to the kalman
		// update rule
		const double pvar = p.stdev * p.stdev;
		const double var = o.stdev * o.stdev;
		double gain = pvar / (pvar + var);
		if( gain != gain )
		    gain = 0.5; // this happens when both stdevs are 0. 
		p.mean = p.mean + gain * (o.mean - p.mean);
		p.stdev = sqrt((1.0-gain)*pvar);
	    }
	    else
	    {
		// convert into a vertical patch element
		//p.mean += p.stdev;
		//p.height = 2 * p.stdev; 
		p.horizontal = false;
	    }
	}

	// if either of the patches is vertical, the result is also going
	// to be vertical
	if( !p.horizontal || !o.horizontal)
	{
	    p.horizontal = false;
	    if( o.mean > p.mean )
	    {
		p.height += ( o.mean - p.mean );
		p.mean = o.mean;
		p.stdev = o.stdev;
	    }

	    const double o_min = o.mean - o.height;
	    const double p_min = p.mean - p.height;
	    if( o_min < p_min )
	    {
		p.height = p.mean - o_min;
	    }
	}
	return true;
    }
    return false;
}

