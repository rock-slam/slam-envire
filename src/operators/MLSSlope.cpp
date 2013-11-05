#include "MLSSlope.hpp"
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/multi_array.hpp>
#include <numeric/PlaneFitting.hpp>

using namespace envire;
using namespace Eigen;

ENVIRONMENT_ITEM_DEF( MLSSlope )

static double const UNKNOWN = -std::numeric_limits<double>::infinity();
    
static void updateGradient(MLSGrid const& mls,
        bool use_stddev,
        boost::multi_array<float,2>& mean_angles,
        boost::multi_array<float,3>& diffs,
        boost::multi_array<int, 2>& count,
        double scale,
        int this_index, int this_x, int this_y,
        int other_index, int other_x, int other_y,
        MLSGrid::const_iterator this_cell)
{
    MLSGrid::const_iterator neighbour_cell = 
        std::max_element( mls.beginCell(other_x,other_y), mls.endCell() );

    if( neighbour_cell != mls.endCell() )
    {
        double z0 = this_cell->mean;
        double z1 = neighbour_cell->mean;
        double stdev0 = 0;
        double stdev1 = 0;
        if (use_stddev)
        {
            stdev0 = this_cell->stdev;
            stdev1 = neighbour_cell->stdev;
        }

        if (z0 > z1)
        {
            std::swap(z0, z1);
            std::swap(stdev0, stdev1);
        }

        double min_z = z0 - stdev0;
        double max_z = z1 + stdev1;

        double step = max_z - min_z;
        
        diffs[this_y][this_x][this_index] = step;
        count[this_y][this_x]++;

        diffs[other_y][other_x][other_index] = step;
        count[other_y][other_x]++;
    }
    else
    {
        diffs[other_y][other_x][this_index] = UNKNOWN;
        diffs[other_y][other_x][other_index] = UNKNOWN;
    }
}


bool MLSSlope::updateAll() 
{
    // this implementation can handle only one input at the moment
    if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
        throw std::runtime_error("MLSSlope needs to have exactly 1 input and 1 output for now. Got " + boost::lexical_cast<std::string>(env->getInputs(this).size()) + " inputs and " + boost::lexical_cast<std::string>(env->getOutputs(this).size()) + "outputs");
    
    Grid<float>& travGrid = *env->getOutput< Grid<float>* >(this);
    MLSGrid const& mls = *env->getInput< MLSGrid* >(this);

    if( mls.getWidth() != travGrid.getWidth() && mls.getHeight() != travGrid.getHeight() )
        throw std::runtime_error("mismatching width and/or height between MLSGradient input and output");
    if( mls.getScaleX() != travGrid.getScaleX() && mls.getScaleY() != travGrid.getScaleY() )
        throw std::runtime_error("mismatching cell scale between MLSGradient input and output");

    // init traversibility grid
    boost::multi_array<float,2>& angles(travGrid.getGridData("mean_slope"));
    boost::multi_array<float,2>& max_steps(travGrid.getGridData("max_step"));
    std::fill(max_steps.data(), max_steps.data() + max_steps.num_elements(), UNKNOWN);
    boost::multi_array<float,2>& corrected_max_steps(travGrid.getGridData("corrected_max_step"));
    boost::multi_array<int,2> counts;
    counts.resize( boost::extents[mls.getHeight()][mls.getWidth()]);
    std::fill(counts.data(), counts.data() + counts.num_elements(), 0);
    travGrid.setNoData(UNKNOWN);

    boost::multi_array<float,3> diffs;
    diffs.resize( boost::extents[mls.getHeight()][mls.getWidth()][8]);
    std::fill(diffs.data(), diffs.data() + diffs.num_elements(), 0);

    size_t width = mls.getWidth(); 
    size_t height = mls.getHeight(); 

    if( width == 0 || height == 0 )
	throw std::runtime_error("MLSSlope needs a grid size greater zero for both width and height.");

    double scalex = mls.getScaleX();
    double scaley = mls.getScaleY();

    double diagonal_scale = sqrt(scalex * scalex + scaley * scaley);

    /** Compute the difference between neighbouring cells.
     *
     * It updates the angles, diffs and counts arrays in the following manner:
     *
     *  - angles[y][x] contains the sum of the algebraic gradient between the
     *    heights of the cell and its neighbour. The difference is -abs(diff) if
     *    x_neighbour < x || y_neighbour < y and abs(diff) otherwise. It gets
     *    converted to angles later
     *  - diffs[y][x][neighbour_index] contains the algebraic difference
     *    between the heights of the cell and its neighbour. The difference is
     *    -abs(diff) if x_neighbour < x || y_neighbour < y
     *    and abs(diff) otherwise.
     *  - counts[y][x] contains the count of valid neighbours that [y][x] has.
     */
    static const int
        BOTTOM_CENTER = 0,
        TOP_CENTER = 1,
        TOP_RIGHT = 2,
        BOTTOM_LEFT = 3,
        CENTER_RIGHT = 4,
        CENTER_LEFT = 5,
        BOTTOM_RIGHT = 6,
        TOP_LEFT = 7;

    for(size_t x=1;x<width;x++)
    {
        for(size_t y=1;y<height-1;y++)
        {
            MLSGrid::const_iterator this_cell = 
                std::max_element( mls.beginCell(x,y), mls.endCell() );
            if (this_cell == mls.endCell())
                continue;

            updateGradient(mls, use_stddev, angles, diffs, counts, scaley,
                    BOTTOM_CENTER, x, y, TOP_CENTER, x, y + 1,
                    this_cell);
            updateGradient(mls, use_stddev, angles, diffs, counts, diagonal_scale,
                    TOP_RIGHT, x, y, BOTTOM_LEFT, x - 1, y - 1,
                    this_cell);
            updateGradient(mls, use_stddev, angles, diffs, counts, scalex,
                    CENTER_RIGHT, x, y, CENTER_LEFT, x - 1, y,
                    this_cell);
            updateGradient(mls, use_stddev, angles, diffs, counts, diagonal_scale,
                    BOTTOM_RIGHT, x, y, TOP_LEFT, x - 1, y + 1,
                    this_cell);            
            
            base::PlaneFitting<double> fitter;
            int count = 0;
            double thisHeight = this_cell->mean;
            for(int xi = -1; xi <= 1; xi++) {
                for(int yi = -1; yi <= 1; yi++) {
                    //skip onw entry
                    if(xi == 0 && yi == 0)
                        continue;

                    count++;
                    const int rx = x + xi;
                    const int ry = y + yi;
                    
                    if((rx < 0) || (rx >= (int) width) || (ry < 0) || (ry >= (int) height) )
                        continue;
                    
                    MLSGrid::const_iterator neighbour_cell = 
                        std::max_element( mls.beginCell(rx,ry), mls.endCell() );

                    if( neighbour_cell != mls.endCell() )
                    {
                        Vector3d input(xi * mls.getScaleX(), yi * mls.getScaleY(), thisHeight - neighbour_cell->mean);
//                         std::cout << x << " " << y << " coords " << input.transpose() << std::endl; 
                        fitter.update(input);
                    }

                }
            }

            
            fitter.update(Vector3d(0,0,0));
            
            if (count < 5)
            {
                angles[y][x] = UNKNOWN;
            }
            else
            {
                Vector3d fit(fitter.getCoeffs());
                const double divider = sqrt(fit.x() * fit.x() + fit.y() * fit.y() + 1);
                angles[y][x] = acos(1 / divider);
            }
            
        }
    }

    // Right now, the angles grid contains gradients. Convert to angles
    for(size_t x=1;x<(width - 1);x++)
    {
        for(size_t y=1;y<(height - 1);y++)
        {
            int count = counts[y][x];
            if (count < 5)
            {
                max_steps[y][x] = UNKNOWN;
                corrected_max_steps[y][x] = UNKNOWN;
                continue;
            }


            double max_step = UNKNOWN;
            double corrected_max_step = UNKNOWN;
            for (int i = 0; i < 8; i += 2)
            {
                double step0 = diffs[y][x][i];
                double step1 = diffs[y][x][i + 1];
                max_step = std::max(max_step, step0);
                max_step = std::max(max_step, step1);
                corrected_max_step = std::max(corrected_max_step, step0 - (step0 + step1) / 4);
                corrected_max_step = std::max(corrected_max_step, step0 - (step0 + step1) * 3 / 4);
            }
            max_steps[y][x] = max_step;
            if (max_step < corrected_step_threshold)
                corrected_max_steps[y][x] = corrected_max_step;
            else
                corrected_max_steps[y][x] = max_step;
        }
    }
    // ... and mark the remaining of the border as UNKNOWN
    for(size_t x=0; x < width; ++x)
    {
        angles[0][x] = UNKNOWN;
        max_steps[0][x] = UNKNOWN;
        corrected_max_steps[0][x] = UNKNOWN;
        angles[height-1][x] = UNKNOWN;
        max_steps[height-1][x] = UNKNOWN;
        corrected_max_steps[height-1][x] = UNKNOWN;
    }
    for(size_t y=0; y < height; ++y)
    {
        angles[y][0] = UNKNOWN;
        max_steps[y][0] = UNKNOWN;
        corrected_max_steps[y][0] = UNKNOWN;
        angles[y][width-1] = UNKNOWN;
        max_steps[y][width-1] = UNKNOWN;
        corrected_max_steps[y][width-1] = UNKNOWN;
    }

    return true;
}
