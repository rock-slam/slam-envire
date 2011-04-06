#include "MLSSlope.hpp"
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/multi_array.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSSlope )

static void updateGradient(MLSGrid const& mls, boost::multi_array<double,2>& angles,
        double scale, int this_x, int this_y, int x, int y, MLSGrid::const_iterator this_cell)
{
    MLSGrid::const_iterator neighbour = 
        std::max_element( mls.beginCell(x,y), mls.endCell() );

    if( neighbour != mls.endCell() )
    {
        double mean0 = this_cell->mean;
        double stdev0 = this_cell->stdev;
        double mean1 = neighbour->mean;
        double stdev1 = neighbour->stdev;

        double g[4];
        g[0] = fabs(mean1 + stdev1 - mean0);
        g[1] = fabs(mean1 + stdev1 - mean0 - stdev0);
        g[2] = fabs(mean1 - mean0 - stdev0);
        g[3] = fabs(mean1 - mean0);
        double gradient = *std::max_element(g, g + 4) / scale;

        angles[this_y][this_x] = std::max(angles[this_y][this_x], gradient);
        angles[y][x] = std::max(angles[y][x], gradient);
    }
}


bool MLSSlope::updateAll() 
{
    // this implementation can handle only one input at the moment
    if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
        throw std::runtime_error("MLSSlope needs to have exactly 1 input and 1 output for now.");
    
    Grid<double>& travGrid = *env->getOutput< Grid<double>* >(this);
    MLSGrid const& mls = *env->getInput< MLSGrid* >(this);

    if( mls.getWidth() != travGrid.getWidth() && mls.getHeight() != travGrid.getHeight() )
        throw std::runtime_error("mismatching width and/or height between MLSGradient input and output");
    if( mls.getScaleX() != travGrid.getScaleX() && mls.getScaleY() != travGrid.getScaleY() )
        throw std::runtime_error("mismatching cell scale between MLSGradient input and output");

    double const UNKNOWN = -std::numeric_limits<double>::infinity();
    
    // init traversibility grid
    boost::multi_array<double,2>& angles(travGrid.getGridData());
    std::fill(angles.data(), angles.data() + angles.num_elements(), UNKNOWN);
    travGrid.setNoData(UNKNOWN);

    size_t width = mls.getWidth(); 
    size_t height = mls.getHeight(); 

    double scalex = mls.getScaleX();
    double scaley = mls.getScaleY();

    double diagonal_scale = sqrt(scalex * scalex + scaley * scaley);
    for(size_t x=1;x<width;x++)
    {
        for(size_t y=1;y<height;y++)
        {
            MLSGrid::const_iterator this_cell = 
                std::max_element( mls.beginCell(x,y), mls.endCell() );
            if (this_cell == mls.endCell())
                continue;

            updateGradient(mls, angles, diagonal_scale, x, y, x - 1, y - 1, this_cell);
            updateGradient(mls, angles, scaley, x, y, x, y - 1, this_cell);
            updateGradient(mls, angles, scalex, x, y, x - 1, y, this_cell);
        }
    }

    // Right now, the angles grid contains gradients. Convert to angles
    for(size_t x=1;x<(width - 1);x++)
    {
        for(size_t y=1;y<(height - 1);y++)
        {
            double gradient = angles[y][x];
            if (gradient != UNKNOWN)
                angles[y][x] = atan(gradient);
        }
    }
    // ... and mark the remaining of the border as UNKNOWN
    for(size_t x=0; x < width; ++x)
        angles[height-1][x] = UNKNOWN;
    for(size_t y=0; y < height; ++y)
        angles[y][width-1] = UNKNOWN;

    return true;
}
