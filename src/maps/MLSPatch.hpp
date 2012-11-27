#ifndef __MULTILEVELSURFACEPATCH_HPP__
#define __MULTILEVELSURFACEPATCH_HPP__

namespace envire
{

/** The representation of one surface in a cell
 *
 * This data structure either represents a surface or a vertical block,
 * depending on the value of the \c horizontal field.
 *
 * If \c horizontal is true, a surface is represented as a mean altitude
 * and standard deviation around that mean.
 *
 * If \c horizontal is false, a vertical block is represented, in which
 * the mean value is the top of the block, and mean - height the bottom
 * of it. The standard deviation still applies.
 */
struct SurfacePatch
{
    friend class MLSGrid;

    /** a surface patch can be three different types,
     * each changing how the cell values are interpreded
     *
     * HORIZONTAL - is a horizontal patch, which does not have a height value
     * VERTICAL - used to represent walls and other vertical structures, has a height
     * NEGATIVE - represent absence of structure, otherwise equal to VERTICAL
     */
    enum TYPE
    {
	VERTICAL = 0,
	HORIZONTAL = 1,
	NEGATIVE = 2
    };

    SurfacePatch() {};
    SurfacePatch( float mean, float stdev, float height = 0, TYPE type = HORIZONTAL )
	: mean(mean), stdev(stdev), height(height), type(type), update_idx(0) {};

    /** Experimental code. Don't use it unless you know what you are
     * doing */
    float distance( const SurfacePatch& other ) const
    {
	if( !isHorizontal() && !other.isHorizontal() )
	    return 0;
	if( !isHorizontal() )
	    return other.mean > mean ?
		other.mean - mean :
		std::max( 0.0f, mean - height - other.mean);
	if( !other.isHorizontal() )
	    return mean > other.mean ?
		mean - other.mean :
		std::max( 0.0f, other.mean - other.height - mean);
	return std::abs( mean - other.mean );
    };

    /** Returns true if the mean of \c self is lower than the mean of \c
     * other
     */
    bool operator<( const SurfacePatch& other ) const
    {
	return mean < other.mean;
    };

    /** The minimum Z extent of this patch
     *
     * The uncertainty is handled by removing sigma_threshold * stdev to
     * the mean minimum Z
     */
    double getMinZ(double sigma_threshold = 2.0) const
    {
	if (isHorizontal())
	    return mean - stdev *  sigma_threshold;
	else
	    return mean - height - stdev * sigma_threshold;
    }

    /** The maximum Z extent of this patch
     *
     * The uncertainty is handled by adding sigma_threshold * stdev to
     * the mean maximum Z
     */
    double getMaxZ(double sigma_threshold = 2.0) const
    {
	return mean + stdev * sigma_threshold;
    }

    /** flag the patch as horizontal
    */
    void setHorizontal()
    {
	type = HORIZONTAL;
    }

    /** flag the patch as vertical
    */
    void setVertical()
    {
	type = VERTICAL;
    }

    void setNegative()
    {
	type = NEGATIVE;
    }

    /** @return true if the patch is horizontal
    */
    bool isHorizontal() const
    {
	return type == HORIZONTAL;
    }

    /** @return true if patch is vertical
    */
    bool isVertical() const
    {
	return type == VERTICAL;
    }

    bool isNegative() const
    {
	return type == NEGATIVE;
    }

    base::Vector3d getColor() const
    {
	return base::Vector3d( color[0], color[1], color[2] ) / 255.0;
    }

    void setColor( const base::Vector3d& c )
    {
	color[0] = c[0] * 255;
	color[1] = c[1] * 255;
	color[2] = c[2] * 255;
    }

    /** The mean Z value. This always represents the top of the patch,
     * regardless whether the patch is horizontal or vertical
     */
    float mean;
    /** The standard deviation in Z */
    float stdev;
    /** For vertical patches, the height of the patch */
    float height;
    /** Horizontal patches are just a mean and standard deviation.
     * Vertical patches also have a height, i.e. the patch is a vertical
     * block between z=(mean-height) and z
     */

    protected:
    TYPE type;

    public:
    size_t update_idx;
    uint8_t color[3];
};

}

#endif
