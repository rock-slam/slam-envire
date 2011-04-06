#include "MLSSlope.hpp"
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/multi_array.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSSlope )

static void updateGradient(MLSGrid const& mls, boost::multi_array<double,2>& angles, boost::multi_array<double,2>& max_step, boost::multi_array<int, 2>& count,
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
        double step = *std::max_element(g, g + 4);
        double gradient = step / scale;

        max_step[this_y][this_x] = std::max(max_step[this_y][this_x], step);
        angles[this_y][this_x] += gradient;
        count[this_y][this_x]++;

        max_step[y][x] = std::max(max_step[y][x], step);
        angles[y][x] += gradient;
        count[y][x]++;
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
    boost::multi_array<double,2>& angles(travGrid.getGridData("mean_slope"));
    std::fill(angles.data(), angles.data() + angles.num_elements(), 0);
    boost::multi_array<double,2>& max_steps(travGrid.getGridData("max_step"));
    std::fill(max_steps.data(), max_steps.data() + max_steps.num_elements(), UNKNOWN);
    boost::multi_array<int,2> counts;
    counts.resize( boost::extents[mls.getHeight()][mls.getWidth()]);
    std::fill(counts.data(), counts.data() + counts.num_elements(), 0);
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

            updateGradient(mls, angles, max_steps, counts, diagonal_scale, x, y, x - 1, y - 1, this_cell);
            updateGradient(mls, angles, max_steps, counts, scaley, x, y, x, y - 1, this_cell);
            updateGradient(mls, angles, max_steps, counts, scalex, x, y, x - 1, y, this_cell);
        }
    }

    // Right now, the angles grid contains gradients. Convert to angles
    for(size_t x=1;x<(width - 1);x++)
    {
        for(size_t y=1;y<(height - 1);y++)
        {
            double gradient = angles[y][x];
            int count = counts[y][x];
            if (count == 0)
                angles[y][x] = UNKNOWN;
            else
                angles[y][x] = atan(gradient / count);
        }
    }
    // ... and mark the remaining of the border as UNKNOWN
    for(size_t x=0; x < width; ++x)
    {
        angles[height-1][x] = UNKNOWN;
        max_steps[height-1][x] = UNKNOWN;
    }
    for(size_t y=0; y < height; ++y)
    {
        angles[y][width-1] = UNKNOWN;
        max_steps[y][width-1] = UNKNOWN;
    }

    return true;
}
