#include "Core.hpp"
#include "Grid.hpp"

#include <iostream>
#include <fstream>
#include <stdint.h>

#include "cpl_string.h"
#include "gdal_priv.h"
#include "ogr_spatialref.h"

using namespace envire;

const std::string Grid::className = "envire::Grid";

const std::string Grid::ELEVATION_MIN = "elevation_min";
const std::string Grid::ELEVATION_MAX = "elevation_max";
const std::string Grid::CONFIDENCE = "confidence";
const std::string Grid::TRAVERSABILITY = "traversability";

Grid::Grid(size_t width, size_t height, double scalex, double scaley) :
    width(width), height(height), scalex(scalex), scaley(scaley)
{
}

Grid::~Grid()
{
    for( std::map<std::string, HolderBase*>::iterator it = data_map.begin();it != data_map.end(); delete((it++)->second) );
}

Grid::Grid(Serialization& so)
    : CartesianMap(so)
{
    so.setClassName(className);
}

void Grid::serialize(Serialization& so)
{
    CartesianMap::serialize(so);
    so.setClassName(className);

    writeMap( getMapFileName(so.getMapPath()) ); 
}

Grid* Grid::clone() 
{
    return new Grid(*this);
}

void Grid::writeMap(const std::string& path)
{
    if( hasData( ELEVATION_MAX ) )
	writeMap(path, ELEVATION_MAX );

    if( hasData( TRAVERSABILITY ) )
	writeMap(path, TRAVERSABILITY );
}

void Grid::writeMap(const std::string& path, const std::string& type )
{
    GByte abyRaster[width*height];

    if( type == ELEVATION_MAX )
    {
	boost::multi_array<double,2> elevation = getGridData<double>( ELEVATION_MAX ); 
	for(int m=0;m<width;m++)
	{
	    for( int n=0;n<height;n++)
	    {
		abyRaster[m+n*width] = std::max(0.0,std::min((elevation[m][n])*51,255.0));
	    }
	}
    }
    else if( type == TRAVERSABILITY )
    {
	boost::multi_array<uint8_t,2> traversability = getGridData<uint8_t>( TRAVERSABILITY ); 
	for(int m=0;m<width;m++)
	{
	    for( int n=0;n<height;n++)
	    {
		abyRaster[m+n*width] = traversability[m][n];
	    }
	}
    }
    else 
    {
	std::cout << "Type " + type + " not supported." << std::endl;
    }

    std::string file = path + "_" + type + ".tiff";
    std::cout << "write file " << file << std::endl;

    GDALAllRegister();

    const char *pszFormat = "GTiff";
    GDALDriver *poDriver;
    char **papszMetadata;

    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

    if( poDriver == NULL )
    {
	std::cout << "driver not found." << std::endl;
    }

    papszMetadata = poDriver->GetMetadata();
    if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
	printf( "Driver %s supports Create() method.\n", pszFormat );
    if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATECOPY, FALSE ) )
	printf( "Driver %s supports CreateCopy() method.\n", pszFormat );

    GDALDataset *poDstDS;       
    char **papszOptions = NULL;

    poDstDS = poDriver->Create( file.c_str(), width, height, 1, GDT_Byte, 
	    papszOptions );

    envire::FrameNode::TransformType t = getEnvironment()->relativeTransform(
	    getFrameNode(),
	    getEnvironment()->getRootNode() );

    Eigen::Matrix4d m = (t * Eigen::Scaling3d(scalex, scaley,0)).matrix();

    double adfGeoTransform[6] = { m(0,3), m(0,0), m(0,1), m(1,3), m(1,0), m(1,1) };

    OGRSpatialReference oSRS;
    char *pszSRS_WKT = NULL;
    GDALRasterBand *poBand;

    poDstDS->SetGeoTransform( adfGeoTransform );

    oSRS.SetUTM( 32, TRUE );
    oSRS.SetWellKnownGeogCS( "WGS84" );
    oSRS.exportToWkt( &pszSRS_WKT );
    poDstDS->SetProjection( pszSRS_WKT );
    CPLFree( pszSRS_WKT );

    poBand = poDstDS->GetRasterBand(1);
    if( type == TRAVERSABILITY )
    {
	GDALColorEntry c1 = {30,30,30,255};
	GDALColorEntry c2 = {250,60,60,255};

	GDALColorTable colorTable;
	colorTable.CreateColorRamp( 0, &c1, 5, &c2 );

	poBand->SetColorTable( &colorTable );
    }
    poBand->RasterIO( GF_Write, 0, 0, width, height, 
	    abyRaster, width, height, GDT_Byte, 0, 0 );    

    GDALClose( (GDALDatasetH) poDstDS );
}

void Grid::readMap(const std::string& path)
{
    // TODO read from ply file
    throw std::runtime_error("not yet implemented");
}

bool Grid::toGrid( double x, double y, size_t& m, size_t& n )
{
    int am = x/scalex;
    int an = y/scaley;
    if( 0 <= am && am < width && 0 <= an && an < height )
    {
	m = am;
	n = an;
	return true;
    }
    else {
	return false;
    }
}

