#include "MLSToGrid.hpp"
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/multi_array.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSToGrid )

MLSToGrid::MLSToGrid()
    : Operator(1, 1)
    , mOutLayerName(ElevationGrid::ELEVATION) {}

MLSToGrid::~MLSToGrid() {}

void MLSToGrid::serialize( Serialization &so )
{
    Operator::serialize(so);
    so.write("out_layer_name", mOutLayerName);
}

void MLSToGrid::unserialize( Serialization &so )
{
    Operator::unserialize(so);
    so.read("out_layer_name", mOutLayerName);
}

void MLSToGrid::setOutput(Grid<double>* map, std::string const& layer_name)
{
    Operator::setOutput(map);
    mOutLayerName = layer_name;
}

bool MLSToGrid::updateAll() 
{
    Grid<double>& travGrid = *env->getOutput< Grid<double>* >(this);
    MLSGrid const& mls = *env->getInput< MLSGrid* >(this);

    if( mls.getWidth() != travGrid.getWidth() && mls.getHeight() != travGrid.getHeight() )
        throw std::runtime_error("mismatching width and/or height between MLSGradient input and output");
    if( mls.getScaleX() != travGrid.getScaleX() && mls.getScaleY() != travGrid.getScaleY() )
        throw std::runtime_error("mismatching cell scale between MLSGradient input and output");

    // init traversibility grid
    size_t width = mls.getWidth(); 
    size_t height = mls.getHeight(); 

    boost::multi_array<double, 2>& out_data = travGrid.getGridData(mOutLayerName);

    for(size_t x=0;x<width;x++)
    {
        for(size_t y=0;y<height;y++)
        {
            MLSGrid::const_iterator this_cell = 
                std::max_element( mls.beginCell(x,y), mls.endCell() );
            if (this_cell == mls.endCell())
                continue;

            out_data[y][x] = this_cell->mean;
        }
    }

    return true;
}
