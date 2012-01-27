#include "GridBase.hpp"
#include "Grid.hpp"

using namespace envire;

const std::string GridBase::className = "envire::GridBase";

GridBase::GridBase()
    : cellSizeX(0), cellSizeY(0), scalex(0), scaley(0), offsetx(0), offsety(0) {}

GridBase::GridBase(size_t cellSizeX, size_t cellSizeY,
        double scalex, double scaley, double offsetx, double offsety)
    : cellSizeX(cellSizeX), cellSizeY(cellSizeY)
    , scalex(scalex), scaley(scaley)
    , offsetx(offsetx), offsety(offsety)
{
}

GridBase::~GridBase()
{
}

GridBase::GridBase(Serialization& so)
    : cellSizeX(0), cellSizeY(0)
    , scalex(0), scaley(0)
    , offsetx(0), offsety(0)
{
    unserialize(so);
}

void GridBase::serialize(Serialization& so)
{
    CartesianMap::serialize(so);

    so.write("cellSizeX", cellSizeX );
    so.write("cellSizeY", cellSizeY );
    so.write("scalex", scalex );
    so.write("scaley", scaley );
    so.write("offsetx", offsetx );
    so.write("offsety", offsety );

    // For backward compatibility with older versions of envire
    so.write("width", cellSizeX );
    so.write("height", cellSizeY );
}

void GridBase::unserialize(Serialization& so)
{
    CartesianMap::unserialize(so);
    
    if (!so.read("cellSizeX", cellSizeX ))
        so.read("width", cellSizeX);
    if (!so.read("cellSizeY", cellSizeY ))
        so.read("height", cellSizeY);

    so.read("scalex", scalex );
    so.read("scaley", scaley );
    so.read("offsetx", offsetx );
    so.read("offsety", offsety );
}

bool GridBase::toGrid( Eigen::Vector3d const& point,
        size_t& xi, size_t& yi, FrameNode const* frame) const
{
    if (frame)
    {
        Eigen::Vector3d transformed = toMap(point, *frame);
        return toGrid(transformed.x(), transformed.y(), xi, yi);
    }
    else
    {
        return toGrid(point.x(), point.y(), xi, yi);
    }
}

bool GridBase::toGrid( double x, double y, size_t& xi, size_t& yi) const
{
    size_t am = floor((x-offsetx)/scalex);
    size_t an = floor((y-offsety)/scaley);
    if( 0 <= am && am < cellSizeX && 0 <= an && an < cellSizeY )
    {
	xi = am;
	yi = an;
	return true;
    }
    else {
	return false;
    }
}

Eigen::Vector3d GridBase::fromGrid(size_t xi, size_t yi, FrameNode const* frame) const
{
    double map_x, map_y;
    fromGrid(xi, yi, map_x, map_y);
    if (frame)
    {
        return fromMap(Eigen::Vector3d(map_x, map_y, 0), *frame);
    }
    else
    {
        return Eigen::Vector3d(map_x, map_y, 0);
    }
}

void GridBase::fromGrid( size_t xi, size_t yi, double& x, double& y) const
{
    x = (xi+0.5) * scalex + offsetx;
    y = (yi+0.5) * scaley + offsety;
}

bool GridBase::toGrid( const Point2D& point, Position& pos) const
{
    return toGrid( point.x(), point.y(), pos.x, pos.y);
}

void GridBase::fromGrid( const Position& pos, Point2D& point) const
{
    fromGrid( pos.x, pos.y, point.x(), point.y());
}

GridBase::Point2D GridBase::fromGrid( const Position& pos) const
{
    Point2D point;
    fromGrid( pos.x, pos.y, point.x(), point.y());
    return point;
}

bool GridBase::contains( const Position& pos ) const
{
    return (pos.x >= 0 && pos.x < cellSizeX 
	    && pos.y >= 0 && pos.y < cellSizeY);
}
        
GridBase::Extents GridBase::getExtents() const
{
    // TODO provide proper extents
    return Extents( Eigen::Vector2d( cellSizeX * scalex, cellSizeY * scaley ) ); 
}

template<typename T>
static GridBase::Ptr readGridFromGdalHelper(std::string const& path, std::string const& band_name, int band)
{
    typename envire::Grid<T>::Ptr result = new Grid<T>();
    result->readGridData(band_name, path, band);
    return result;
}

GridBase::Ptr GridBase::readGridFromGdal(std::string const& path, std::string const& band_name, int band)
{
    GDALDataset  *poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen(path.c_str(), GA_ReadOnly );
    if( poDataset == NULL )
        throw std::runtime_error("can not open file " + path);

    GDALRasterBand  *poBand;
    if(poDataset->GetRasterCount() < band)
    {
        std::stringstream strstr;
        strstr << "file " << path << " has " << poDataset->GetRasterCount() 
            << " raster bands but the band " << band << " is required";
        throw std::runtime_error(strstr.str());
    }

    poBand = poDataset->GetRasterBand(band);
    switch(poBand->GetRasterDataType())
    {
    case  GDT_Byte:
        return readGridFromGdalHelper<uint8_t>(path, band_name, band);
    case GDT_Int16:
        return readGridFromGdalHelper<int16_t>(path, band_name, band);
    case GDT_UInt16:
        return readGridFromGdalHelper<uint16_t>(path, band_name, band);
    case GDT_Int32:
        return readGridFromGdalHelper<int32_t>(path, band_name, band);
    case GDT_UInt32:
        return readGridFromGdalHelper<uint32_t>(path, band_name, band);
    case GDT_Float32:
        return readGridFromGdalHelper<float>(path, band_name, band);
    case GDT_Float64:
        return readGridFromGdalHelper<double>(path, band_name, band);
    default:
        throw std::runtime_error("enview::Grid<T>: GDT type is not supported.");  
    }
}

void GridBase::copyBandFrom(GridBase const& source, std::string const& source_band, std::string const& _target_band)
{
    throw std::runtime_error("copyBandFrom is not implemented for this type of grid");
}

