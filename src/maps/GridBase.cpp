#include "GridBase.hpp"
#include "Grid.hpp"
#include <envire/tools/BresenhamLine.hpp>

using namespace envire;

const std::string GridBase::className = "envire::GridBase";

GridBase::GridBase(std::string const& id)
    : Map<2>(id)
    , cellSizeX(0), cellSizeY(0), scalex(0), scaley(0), offsetx(0), offsety(0) {}

GridBase::GridBase(size_t cellSizeX, size_t cellSizeY,
        double scalex, double scaley, double offsetx, double offsety,
        std::string const& id)
    : Map<2>(id)
    , cellSizeX(cellSizeX), cellSizeY(cellSizeY)
    , scalex(scalex), scaley(scaley)
    , offsetx(offsetx), offsety(offsety)
{
}

GridBase::~GridBase()
{
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

bool envire::GridBase::getRectPoints(const base::Pose2D &pose, double width, double height, GridBase::Position &upLeft_g, GridBase::Position &upRight_g, GridBase::Position &downLeft_g, GridBase::Position &downRight_g) const
{
    const double widthHalf = width / 2.0;
    const double heightHalf = height / 2.0;
    Eigen::Rotation2D<double> rot(pose.orientation);
    
    Eigen::Vector2d upLeft = pose.position +  rot * Eigen::Vector2d(-widthHalf, heightHalf);
    Eigen::Vector2d upRight = pose.position +  rot * Eigen::Vector2d(widthHalf, heightHalf);
    Eigen::Vector2d downRight = pose.position +  rot * Eigen::Vector2d(widthHalf, -heightHalf);
    Eigen::Vector2d downLeft = pose.position +  rot * Eigen::Vector2d(-widthHalf, -heightHalf);

    if(upLeft.y() < downLeft.y())
    {
        std::swap(upLeft, downLeft);
        std::swap(upRight, downRight);
    }

    if(upRight.x() < upLeft.x())
    {
        std::swap(upLeft, upRight);
        std::swap(downLeft, downRight);
    }

//     std::cout << "P UL:" << upLeft.transpose() << std::endl;
//     std::cout << "P UR:" << upRight.transpose() << std::endl;
//     std::cout << "P DR:" << downRight.transpose() << std::endl;
//     std::cout << "P DL:" << downLeft.transpose() << std::endl;

    if(!toGrid(upLeft, upLeft_g))
    {
//         std::cout << "Error P UL:" << upLeft.transpose() << " outside of grid " << std::endl;
        return false;
    }
    if(!toGrid(upRight, upRight_g))
    {
//         std::cout << "Error P UR:" << upRight.transpose() << "  outside of grid " << std::endl;
        return false;
    }
    if(!toGrid(downLeft, downLeft_g))
    {
//         std::cout << "Error P DL:" << downLeft.transpose() << "  outside of grid " << std::endl;
        return false;
    }
    if(!toGrid(downRight, downRight_g))
    {
//         std::cout << "Error P DR:" << downRight.transpose() << "  outside of grid " << std::endl;
        return false;
    }
 
//     std::cout << "upLeftOut    X " << upLeft_g.x << " Y " << upLeft_g.y << std::endl;
//     std::cout << "upRightOut   X " << upRight_g.x << " Y " << upRight_g.y << std::endl;
//     std::cout << "downLeftOut  X " << downLeft_g.x << " Y " << downLeft_g.y << std::endl;
//     std::cout << "downRightOut X " << downRight_g.x << " Y " << downRight_g.y << std::endl;
 
    return true;
}


bool envire::GridBase::forEachInRectangles(const base::Pose2D &rectCenter, double innerWidth_w, double innerHeight_w, boost::function< void (size_t, size_t)> innerCallback, 
                                           double outerWidth_w, double outerHeight_w, boost::function< void (size_t, size_t)> outerCallback) const
{
    envire::GridBase::Position upLeft_g;
    envire::GridBase::Position upRight_g;
    envire::GridBase::Position downLeft_g;
    envire::GridBase::Position downRight_g;
    
    if(!getRectPoints(rectCenter, innerWidth_w, innerHeight_w, upLeft_g, upRight_g, downLeft_g, downRight_g))
        return false;

    envire::GridBase::Position upLeftOut;
    envire::GridBase::Position upRightOut;
    envire::GridBase::Position downLeftOut;
    envire::GridBase::Position downRightOut;
    
    if(!getRectPoints(rectCenter, outerWidth_w, outerHeight_w, upLeftOut, upRightOut, downLeftOut, downRightOut))
        return false;

//     std::cout << "upLeftOut    X " << upLeftOut.x << " Y " << upLeftOut.y << std::endl;
//     std::cout << "upRightOut   X " << upRightOut.x << " Y " << upRightOut.y << std::endl;
//     std::cout << "downLeftOut  X " << downLeftOut.x << " Y " << downLeftOut.y << std::endl;
//     std::cout << "downRightOut X " << downRightOut.x << " Y " << downRightOut.y << std::endl;
    
    std::vector<envire::GridBase::Position> leftIn;
    std::vector<envire::GridBase::Position> rightIn;
    std::vector<envire::GridBase::Position> leftOut;
    std::vector<envire::GridBase::Position> rightOut;

    if(upLeft_g.y > upRight_g.y)
    {
        //points of left side
        lineBresenham(upLeft_g, downLeft_g, leftIn);
        lineBresenham(downLeft_g, downRight_g, leftIn);

        lineBresenham(upLeftOut, downLeftOut, leftOut);
        lineBresenham(downLeftOut, downRightOut, leftOut);

        //points of right side
        lineBresenham(upLeft_g, upRight_g, rightIn);
        lineBresenham(upRight_g, downRight_g, rightIn);

        lineBresenham(upLeftOut, upRightOut, rightOut);
        lineBresenham(upRightOut, downRightOut, rightOut);
    }
    else
    {
        //points of left side
        lineBresenham(upRight_g, upLeft_g, leftIn);
        lineBresenham(upLeft_g, downLeft_g, leftIn);

        lineBresenham(upRightOut, upLeftOut, leftOut);
        lineBresenham(upLeftOut, downLeftOut, leftOut);

        //points of right side
        lineBresenham(upRight_g, downRight_g, rightIn);
        lineBresenham(downRight_g, downLeft_g, rightIn);

        lineBresenham(upRightOut, downRightOut, rightOut);
        lineBresenham(downRightOut, downLeftOut, rightOut);
    }

//     std::cout << "Left " << std::endl;
//     for(std::vector<envire::GridBase::Position>::const_iterator it = leftOut.begin(); it != leftOut.end(); it++ )
//     {
//         std::cout << "X " << it->x << " Y " << it->y << std::endl;
//     }
//     std::cout << "Right " << std::endl;
//     for(std::vector<envire::GridBase::Position>::const_iterator it = rightOut.begin(); it != rightOut.end(); it++ )
//     {
//         std::cout << "X " << it->x << " Y " << it->y << std::endl;
//     }
    
    std::vector<envire::GridBase::Position>::const_iterator leftInIt = leftIn.begin();
    std::vector<envire::GridBase::Position>::const_iterator rightInIt = rightIn.begin();
    std::vector<envire::GridBase::Position>::const_iterator leftOutIt = leftOut.begin();
    std::vector<envire::GridBase::Position>::const_iterator rightOutIt = rightOut.begin();

    
    size_t inY;
    size_t outY;
    size_t minX, maxX;
    size_t minXOut, maxXOut;
    while((leftOutIt != leftOut.end()) && (rightOutIt != rightOut.end()))
    {
        if(leftInIt != leftIn.end())
            inY = leftInIt->y;
        
        outY = leftOutIt->y;

        maxX = 0;
        minX = std::numeric_limits<size_t>::max();
        maxXOut = 0;
        minXOut = std::numeric_limits<size_t>::max();
        
        while((leftInIt != leftIn.end()) && (leftInIt->y == outY))
        {
            if(minX > leftInIt->x)
                minX = leftInIt->x;
            if(maxX < leftInIt->x)
                maxX = leftInIt->x;
            
            leftInIt++;
        }

        while((rightInIt != rightIn.end()) && (rightInIt->y == outY))
        {
            if(minX > rightInIt->x)
                minX = rightInIt->x;
            if(maxX < rightInIt->x)
                maxX = rightInIt->x;
            
            rightInIt++;
        }

        while((leftOutIt != leftOut.end()) && (leftOutIt->y == outY))
        {
            if(minXOut > leftOutIt->x)
                minXOut = leftOutIt->x;
            if(maxXOut < leftOutIt->x)
                maxXOut = leftOutIt->x;
            
            leftOutIt++;
        }

        while((rightOutIt != rightOut.end()) && (rightOutIt->y == outY))
        {
            if(minXOut > rightOutIt->x)
                minXOut = rightOutIt->x;
            if(maxXOut < rightOutIt->x)
                maxXOut = rightOutIt->x;
            
            rightOutIt++;
        }
        
        if(inY == outY)
        {
//             std::cout << "minX " << minX << " maxX " << maxX << " minXOut " << minXOut << " maxXOut " << maxXOut << std::endl;  
            for(size_t x = minXOut; x < minX; x++)
            {
                outerCallback(x, outY);
            }
            for(size_t x = minX; x < maxX; x++)
            {
                innerCallback(x, outY);
            }
            for(size_t x = maxX; x < maxXOut; x++)
            {
                outerCallback(x, outY);
            }

        }
        else
        {
//             std::cout << "NE minX " << minX << " maxX " << maxX << " minXOut " << minXOut << " maxXOut " << maxXOut << std::endl;  
            for(size_t x = minXOut; x < maxXOut; x++)
            {
                outerCallback(x, outY);
            }        
        }
        
    }
    return true;

}

bool envire::GridBase::forEachInRectangle(base::Pose2D pose, double width, double height, boost::function<void (size_t, size_t)> callbackGrid) const
{
    envire::GridBase::Position ulGrid;
    envire::GridBase::Position urGrid;
    envire::GridBase::Position dlGrid;
    envire::GridBase::Position drGrid;
    
    if(!getRectPoints(pose, width, height, ulGrid, urGrid, dlGrid, drGrid))
        return false;

    std::vector<envire::GridBase::Position> left;
    std::vector<envire::GridBase::Position> right;

    if(ulGrid.y > urGrid.y)
    {
        //points of left side
        lineBresenham(ulGrid, dlGrid, left);
        lineBresenham(dlGrid, drGrid, left);

        //points of right side
        lineBresenham(ulGrid, urGrid, right);
        lineBresenham(urGrid, drGrid, right);
    }
    else
    {
        //points of left side
        lineBresenham(urGrid, ulGrid, left);
        lineBresenham(ulGrid, dlGrid, left);

        //points of right side
        lineBresenham(urGrid, drGrid, right);
        lineBresenham(drGrid, dlGrid, right);
    }
    
    std::vector<envire::GridBase::Position>::const_iterator leftIt = left.begin();
    std::vector<envire::GridBase::Position>::const_iterator rightIt = right.begin();

    size_t curY;
    size_t minX, maxX;
    while((leftIt != left.end()) && (rightIt != right.end()))
    {
        curY = leftIt->y;

        maxX = 0;
        minX = std::numeric_limits<size_t>::max();
        
        assert(rightIt->y == leftIt->y);
        
        while((leftIt != left.end()) && (leftIt->y == curY))
        {
            if(minX > leftIt->x)
                minX = leftIt->x;
            if(maxX < leftIt->x)
                maxX = leftIt->x;
            
            leftIt++;
        }

        while((rightIt != right.end()) && (rightIt->y == curY))
        {
            if(minX > rightIt->x)
                minX = rightIt->x;
            if(maxX < rightIt->x)
                maxX = rightIt->x;
            
            rightIt++;
        }

        for(size_t x = minX; x <= maxX; x++)
        {
            callbackGrid(x, curY);
        }        
    }
    return true;
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
    Eigen::Vector2d min( offsetx, offsety );
    return Extents( min, min + Eigen::Vector2d( cellSizeX * scalex, cellSizeY * scaley ) ); 
}

template<typename T>
static GridBase::Ptr readGridFromGdalHelper(std::string const& path, std::string const& band_name, int band)
{
    typename envire::Grid<T>::Ptr result = new Grid<T>();
    result->readGridData(band_name, path, band);
    return result;
}

std::pair<GridBase::Ptr, envire::Transform> GridBase::readGridFromGdal(std::string const& path, std::string const& band_name, int band)
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

    double adfGeoTransform[6];
    poDataset->GetGeoTransform(adfGeoTransform);  
    double offsetx = adfGeoTransform[0];
    double offsety = adfGeoTransform[3];

    GridBase::Ptr map;
    switch(poBand->GetRasterDataType())
    {
    case  GDT_Byte:
        map =  readGridFromGdalHelper<uint8_t>(path, band_name, band);
        break;
    case GDT_Int16:
        map =  readGridFromGdalHelper<int16_t>(path, band_name, band);
        break;
    case GDT_UInt16:
        map =  readGridFromGdalHelper<uint16_t>(path, band_name, band);
        break;
    case GDT_Int32:
        map =  readGridFromGdalHelper<int32_t>(path, band_name, band);
        break;
    case GDT_UInt32:
        map =  readGridFromGdalHelper<uint32_t>(path, band_name, band);
        break;
    case GDT_Float32:
        map =  readGridFromGdalHelper<float>(path, band_name, band);
        break;
    case GDT_Float64:
        map =  readGridFromGdalHelper<double>(path, band_name, band);
        break;
    default:
        throw std::runtime_error("enview::Grid<T>: GDT type is not supported.");  
    }

    if (adfGeoTransform[1] < 0)
        offsetx -= map->getCellSizeX() * map->getScaleX();
    if (adfGeoTransform[5] < 0)
        offsety -= map->getCellSizeY() * map->getScaleY();

    Transform transform =
        Transform(Eigen::Translation<double, 3>(offsetx, offsety, 0));
    return std::make_pair(map, transform);
}

void GridBase::copyBandFrom(GridBase const& source, std::string const& source_band, std::string const& _target_band)
{
    throw std::runtime_error("copyBandFrom is not implemented for this type of grid");
}

GridBase::Ptr GridBase::create(std::string const& type_name,
        size_t cellSizeX, size_t cellSizeY,
        double scale_x, double scale_y,
        double offset_x, double offset_y)
{
    Serialization so;
    so.begin();
    so.write("class", type_name);
    so.write("id", -1);
    so.write("label", "");
    so.write("immutable", false);
    so.write("cellSizeX", cellSizeX);
    so.write("cellSizeY", cellSizeY);
    so.write("scalex", scale_x);
    so.write("scaley", scale_y);
    so.write("offsetx", offset_x);
    so.write("offsety", offset_y);
    so.write("map_count", 0);
    EnvironmentItem::Ptr ptr = SerializationFactory::createObject(type_name, so);
    so.end();
    ptr->setUniqueId("");
    return boost::dynamic_pointer_cast<GridBase>(ptr);
}

bool GridBase::isCellAlignedWith(GridBase const& grid) const
{
    if (getScaleX() != grid.getScaleX() ||
        getScaleY() != grid.getScaleY() )
	return false;

    // see if the difference of the world position of the 0 grid cells is a
    // multiple of the cell scale
    Eigen::Vector3d rootDiff = 
	fromGrid( 0, 0, getEnvironment()->getRootNode() )
	 -grid.fromGrid( 0, 0, grid.getEnvironment()->getRootNode() );
    Eigen::Vector3d cellDiff = 
	(rootDiff.array() * Eigen::Array3d( 1.0/getScaleX(), 1.0/getScaleY(), 0 )).matrix();

    if( !(cellDiff.cast<int>().cast<double>() - cellDiff).isZero() )
	return false;

    return true;
}

bool GridBase::isAlignedWith(GridBase const& grid) const
{
    if (getCellSizeX() != grid.getCellSizeX() ||
        getCellSizeY() != grid.getCellSizeY() ||
        getOffsetX() != grid.getOffsetX() ||
        getOffsetY() != grid.getOffsetY())
        return false;

    Transform tf = getEnvironment()->relativeTransform(this, &grid);
    base::Vector3d p(grid.getCellSizeX(), grid.getCellSizeY(), 0);
    p = tf * p;
    if (rint(p.x()) != grid.getCellSizeX() ||
            rint(p.y()) != grid.getCellSizeY())
        return false;

    return true;
}

