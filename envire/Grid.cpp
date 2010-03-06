#include "Core.hpp"
#include "Grid.hpp"

#include <iostream>
#include <fstream>

using namespace envire;

const std::string Grid::className = "envire::Grid";

const std::string Grid::ELEVATION = "elevation";
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

    writeMap( so.getMapPath() ); 
}

Grid* Grid::clone() 
{
    return new Grid(*this);
}

void Grid::writeMap(const std::string& path)
{
    std::string file = path + ".elev";
    std::ofstream data(file.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + file + "' for writing.");
    }

    boost::multi_array<double,2> elevation = getData<double>( ELEVATION ); 

    for(int m=0;m<width;m++)
    {
	for( int n=0;n<height;n++)
	{
	    data << m*scalex << " " << n*scaley << " " <<  elevation[m][n] << std::endl;
	}
	data << std::endl;
    }
}

void Grid::readMap(const std::string& path)
{
    // TODO read from ply file
    throw std::runtime_error("not yet implemented");
}

bool Grid::hasData(const std::string& type)
{
    return data_map.count(type);
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

