#include "MLSGrid.hpp"
#include <fstream>
#include <limits>
#include "tools/Numeric.hpp"
#include <algorithm>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSGrid )
/** For backward compatibility reasons, we have to export MLSGrid as
 * MultiLevelSurfaceGrid as well
 */
static SerializationPlugin<MLSGrid> factory("MultiLevelSurfaceGrid");

MLSGrid::MLSGrid(size_t cellSizeX, size_t cellSizeY, double scalex, double scaley, double offsetx, double offsety)
    : GridBase( cellSizeX, cellSizeY, scalex, scaley, offsetx, offsety )
    , cells( boost::extents[cellSizeX][cellSizeY] )
    , gapSize( 1.0 ), thickness( 0.05 ), cellcount( 0 )
    , hasCellColor_(false), mem_pool( sizeof( SurfacePatchItem ) )
{
    clear();
}

void MLSGrid::clear()
{
    for(size_t xi=0;xi<cellSizeX;xi++)
    {
	for(size_t yi=0;yi<cellSizeY;yi++)
	{
	    cells[xi][yi] = NULL;
	}
    }
    mem_pool.purge_memory();
    cellcount = 0;
    if(index) index->reset();
    extents = Extents();
    hasCellColor_ = false;
}

MLSGrid::MLSGrid(const MLSGrid& other)
    : GridBase( other )
    , cells( boost::extents[other.cellSizeX][other.cellSizeY] )
    , gapSize( other.gapSize ), thickness( other.thickness ), cellcount( other.cellcount )
    , hasCellColor_( other.hasCellColor_), mem_pool( sizeof( SurfacePatchItem ) )
{
    clear();
    for(size_t xi=0;xi<cellSizeX;xi++)
    {
	for(size_t yi=0;yi<cellSizeY;yi++)
	{
	    for( const_iterator it = other.beginCell( xi,yi ); it != other.endCell(); it++ )
		insertTail( xi, yi, *it );
	}
    }
}

MLSGrid& MLSGrid::operator=(const MLSGrid& other)
{
    if( this != &other )
    {
	GridBase::operator=(other);

	mem_pool.purge_memory();
	cells.resize( boost::extents[cellSizeX][cellSizeY] );

	for(size_t xi=0;xi<cellSizeX;xi++)
	{
	    for(size_t yi=0;yi<cellSizeY;yi++)
	    {
		cells[xi][yi] = NULL;
		for( const_iterator it = other.beginCell( xi,yi ); it != endCell(); it++ )
		    insertTail( xi, yi, *it );
	    }
	}
    }

    gapSize = other.gapSize;
    thickness = other.thickness;
    cellcount = other.cellcount;

    return *this;
}

envire::MLSGrid* MLSGrid::cloneShallow() const
{
    MLSGrid* res = new MLSGrid( cellSizeX, cellSizeY, scalex, scaley );
    res->gapSize = gapSize;
    res->thickness = thickness;
    res->cellcount = cellcount;
    return res;
}

MLSGrid::MLSGrid(Serialization& so)
    : mem_pool( sizeof( SurfacePatchItem ) )
{
    unserialize(so);
}

MLSGrid::~MLSGrid()
{
}

void MLSGrid::serialize(Serialization& so)
{
    GridBase::serialize(so);

    so.write( "hasCellColor", hasCellColor_ );
    writeMap( so.getBinaryOutputStream(getMapFileName() + ".mls") );
}

void MLSGrid::unserialize(Serialization& so)
{
    GridBase::unserialize(so);

    if( so.hasKey( "hasCellColor" ) )
	so.read( "hasCellColor", hasCellColor_ );
    else
	hasCellColor_ = false;

    cells.resize( boost::extents[cellSizeX][cellSizeY] );
    try { readMap( so.getBinaryInputStream(getMapFileName() + ".mls") ); }
    catch(...)
    {
        // Try with MultiLevelSurfaceGrid (backward compatibility)
        try {
	    readMap( so.getBinaryInputStream(getMapFileName("envire::MultiLevelSurfaceGrid") + ".mls"));
	    return;
	}
        catch(Serialization::NoSuchBinaryStream)
        { 
            // Assume that serialization is being used as a factory
            return;
        }
        catch(...) { throw; }
    }
}

// memory structure of version 1.0
struct SurfacePatchStore10
{
    double mean;
    double stdev;
    double height;
    bool horizontal;
    size_t update_idx;

    size_t xi, yi;

    MLSGrid::SurfacePatch toSurfacePatch()
    {
	MLSGrid::SurfacePatch p( mean, stdev, height, horizontal );
	p.update_idx = update_idx;
	return p;
    }
};

// memory structure of version 1.1
struct SurfacePatchStore11
{
    double mean;
    double stdev;
    double height;
    bool horizontal;
    size_t update_idx;
    base::Vector3d color;

    size_t xi, yi;

    MLSGrid::SurfacePatch toSurfacePatch()
    {
	MLSGrid::SurfacePatch p( mean, stdev, height, horizontal );
	p.update_idx = update_idx;
	p.color = color;
	return p;
    }
};

struct SurfacePatchStore : MLSGrid::SurfacePatch
{
    SurfacePatchStore() {};

    SurfacePatchStore( const MLSGrid::SurfacePatch& data, size_t xi, size_t yi )
	: SurfacePatch(data), xi(xi), yi(yi) {}

    size_t xi, yi;
};

void MLSGrid::writeMap(std::ostream& os)
{
    os << "mls" << std::endl;
    os << "1.1" << std::endl;
    os << sizeof( SurfacePatchStore ) << std::endl;
    os << "bin" << std::endl;

    for(size_t xi=0;xi<cellSizeX;xi++)
    {
	for(size_t yi=0;yi<cellSizeY;yi++)
	{
	    for( iterator it = beginCell( xi,yi ); it != endCell(); it++ )
	    {
		SurfacePatchStore d( *it, xi, yi );
		os.write( reinterpret_cast<const char*>(&d), sizeof( SurfacePatchStore ) );
	    }
	}
    }
}

void MLSGrid::readMap(std::istream& is)
{   
    char c[32];
    is.getline(c, 20);
    if( std::string(c) != "mls" )
	throw std::runtime_error("bad magic " + std::string(c));

    is.getline(c, 20);
    std::string version = std::string(c);
    if( version != "1.0" && version != "1.1" )
	throw std::runtime_error("version not supported " + version );

    is.getline(c, 20);
    if( version == "1.0" )
    {
	if( boost::lexical_cast<int>(std::string(c)) != sizeof( SurfacePatchStore10 ) )
	    throw std::runtime_error("binary size mismatch");
    }
    else if( version == "1.1" )
    {
	if( boost::lexical_cast<int>(std::string(c)) != sizeof( SurfacePatchStore11 ) )
	    throw std::runtime_error("binary size mismatch");
    }

    is.getline(c, 20);
    if( std::string(c) != "bin" )
	throw std::runtime_error("missing bin identifier" + std::string(c));

    if( version == "1.0" )
    {
	SurfacePatchStore10 d;
	while( is.read(reinterpret_cast<char*>(&d), sizeof( SurfacePatchStore10 ) ) )
	{
	    insertTail( d.xi, d.yi, d.toSurfacePatch() );
	}
    }
    else if( version == "1.1" )
    {
	SurfacePatchStore11 d;
	while( is.read(reinterpret_cast<char*>(&d), sizeof( SurfacePatchStore11 ) ) )
	{
	    insertTail( d.xi, d.yi, d.toSurfacePatch() );
	}
    }
}

MLSGrid::iterator MLSGrid::beginCell( size_t xi, size_t yi )
{
    return iterator( cells[xi][yi] );
}

MLSGrid::const_iterator MLSGrid::beginCell( size_t xi, size_t yi ) const
{
    return const_iterator( cells[xi][yi] );
}

MLSGrid::iterator MLSGrid::endCell()
{
    return iterator();
}

MLSGrid::const_iterator MLSGrid::endCell() const
{
    return const_iterator();
}

void MLSGrid::insertHead( size_t xi, size_t yi, const SurfacePatch& value )
{
    SurfacePatchItem* n_item = static_cast<SurfacePatchItem*>(mem_pool.malloc());
    static_cast<SurfacePatch&>(*n_item).operator=(value);
    n_item->next = cells[xi][yi];
    n_item->pthis = &cells[xi][yi];

    cells[xi][yi] = n_item;
    addCell( Position( xi, yi ) );
}

void MLSGrid::insertTail( size_t xi, size_t yi, const SurfacePatch& value )
{
    iterator last, it;
    last = it = beginCell( xi, yi );
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
	cells[xi][yi] = n_item;
	n_item->pthis = &cells[xi][yi];
    }

    addCell( Position( xi, yi ) );
}

MLSGrid::iterator MLSGrid::erase( iterator position )
{
    SurfacePatchItem* &p( position.m_item );
    iterator res( p->next );

    *p->pthis = p->next;
    if( p->next )
	p->next->pthis = p->pthis; 

    cellcount--;
    mem_pool.free( p );

    return res; 
}

MLSGrid::SurfacePatch* MLSGrid::get( const Position& position, const SurfacePatch& patch, double sigma_threshold )
{
    MLSGrid::iterator it = beginCell(position.x, position.y);
    while( it != endCell() )
    {
	MLSGrid::SurfacePatch &p(*it);
	const double interval = sqrt(sq(patch.stdev) + sq(p.stdev)) * sigma_threshold;
	if( p.distance( patch ) < interval )
	{
	    return &p;
	}
	it++;
    }
    return NULL;
}


bool MLSGrid::get(const Eigen::Vector3d& position, double& zpos, double& zstdev)
{
    size_t xi, yi;
    if( toGrid(position.x(), position.y(), xi, yi) )
    {
	SurfacePatch patch( position.z(), zstdev ); 
	SurfacePatch *p = get( Position(xi, yi), patch );
	if( p )
	{
	    zpos = p->mean;
	    zstdev = p->stdev;
	    return true;
	}
    }
    return false;
}

void MLSGrid::updateCell( size_t xi, size_t yi, double mean, double stdev )
{
    updateCell( xi, yi, SurfacePatch( mean, stdev ) );
}

void MLSGrid::updateCell( size_t xi, size_t yi, const SurfacePatch& o )
{
    typedef std::list<MLSGrid::iterator> iterator_list;
    iterator_list merged;

    for(MLSGrid::iterator it = beginCell( xi, yi ); it != endCell(); it++ )
    {
	// merge the patches and remember the ones which where merged 
	if( mergePatch( *it, o ) )
	    merged.push_back( it );
    }

    if( merged.empty() )
    {
	// insert the patch since we didn't merge it with any other
	insertHead( xi, yi, o );
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

bool MLSGrid::mergePatch( SurfacePatch& p, const SurfacePatch& o )
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
		kalman_update( p.mean, p.stdev, o.mean, o.stdev );
		/*
		// for horizontal patches, perform an update similar to the kalman
		// update rule
		const double pvar = p.stdev * p.stdev;
		const double var = o.stdev * o.stdev;
		double gain = pvar / (pvar + var);
		if( gain != gain )
		    gain = 0.5; // this happens when both stdevs are 0. 
		p.mean = p.mean + gain * (o.mean - p.mean);
		p.stdev = sqrt((1.0-gain)*pvar);
		*/
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
	p.update_idx = o.update_idx;

	if( hasCellColor_ )
	    p.color = (p.color + o.color)/2.0;

	return true;
    }
    return false;
}

std::pair<MLSGrid::SurfacePatch*, double> 
    getNearestPatch( const MLSGrid::SurfacePatch& p, MLSGrid::iterator begin, MLSGrid::iterator end )
{
    MLSGrid::SurfacePatch* min = NULL;
    double dist = std::numeric_limits<double>::infinity();

    // find the cell with the smallest z-diff
    while( begin != end )
    {
	double d;
	if( (d = p.distance( *begin )) < dist )
	{
	    min = &(*begin);
	    dist = d;
	}
	begin++;
    }

    return std::make_pair( min, dist );
}

std::pair<double, double> MLSGrid::matchHeight( const MLSGrid& other )
{
    assert( other.getWidth() == getWidth() && other.getHeight() == getHeight() );

    double d1=0, d2=0;

    for(size_t xi=0;xi<cellSizeX;xi++)
    {
	for(size_t yi=0;yi<cellSizeY;yi++)
	{
	    if( other.cells[xi][yi] && cells[xi][yi] )
	    {
		for( const_iterator it = other.beginCell(xi,yi); it != other.endCell(); it++ )
		{
		    const SurfacePatch &p( *it );
		    std::pair<SurfacePatch*,double> res = getNearestPatch( p, beginCell(xi,yi), endCell() );

		    const double diff = res.second;
		    const double var = sq( res.first->stdev ) + sq( p.stdev );

		    d1 += diff / var;
		    d2 += 1.0 / var;
		}
	    }
	}
    }

    double delta = d1 / d2;
    double var = 1.0 / d2;

    return std::make_pair( delta, var );
}

void MLSGrid::addCell( const Position& pos )
{
    cellcount++;

    if( index )
	index->addCell(pos);

    Point2D p;
    fromGrid( pos, p );
    extents.extend( p );
}

void MLSGrid::initIndex()
{
   index = boost::shared_ptr<Index>( new Index() ); 
}
