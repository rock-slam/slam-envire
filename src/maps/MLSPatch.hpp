#ifndef __MULTILEVELSURFACEPATCH_HPP__
#define __MULTILEVELSURFACEPATCH_HPP__

#include <envire/tools/Numeric.hpp>
#include <envire/maps/MLSConfiguration.hpp>
#include <numeric/PlaneFitting.hpp>

namespace envire
{

template <class T>
inline bool overlap( T a1, T a2, T b1, T b2 )
{
    return 
	((a1 < b2) && (a2 > b2)) ||
	((a1 < b1) && (a2 > b1));
}


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
	: mean(mean), stdev(stdev), height(height), 
	min(mean), max(mean),
	n(1.0),
	normsq(1.0/pow(stdev,4)),
	update_idx(0), 
	type(type) 
	{
	    plane.n = 1.0/pow(stdev,2);
	    plane.z = mean * plane.n;
	    plane.zz = pow(mean,2) * plane.n;
	};

    SurfacePatch( const Eigen::Vector3f &p, float stdev )
	: mean(p.z()), stdev(stdev), height(0),
        plane( p, 1.0f/pow(stdev,2)),  
	min(p.z()), max(p.z()),
	n(1.0), 
	normsq(1.0/pow(stdev,4)),
	update_idx(0),
	type( HORIZONTAL )
	{
	    updatePlane();
	};

    void updateSum() 
    {
	mean = plane.z / plane.n;
	float norm = plane.n / ( pow(plane.n,2) - normsq );
        if( n > 1 )
        {
            float var = 
                std::max(1e-6f, (float)((plane.zz - (pow(mean,2)*(plane.n - 2.0))) * norm - n/plane.n));
            stdev = sqrt(var);
        }
        else
            stdev = sqrt(1.0/plane.n);
    }

    void updatePlane()
    {
	if( n <=3 )
	{
	    updateSum();
	    return;
	}
	base::PlaneFitting<float>::Result res = plane.solve();
	mean = res.getCoeffs()[2];
	float norm = plane.n / ( pow(plane.n,2) - 3.0*normsq );
	float var = std::max(1e-6f, (float)((res.getResiduals()) * norm));
	stdev = sqrt(var);
    }

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

    float getHeight( const Eigen::Vector2f& pos ) const
    {
	// todo do some caching
	float zpos = Eigen::Vector3f( pos.x(), pos.y(), 1.0 ).dot( plane.getCoeffs() );
	return zpos;
    }

    double getSlope() const
    {
        return acos(getNormal().dot(Eigen::Vector3f::UnitZ()));
    }
    
    Eigen::Vector3f getNormal() const
    {
	return Eigen::Vector3f( -plane.getCoeffs().x(), -plane.getCoeffs().y(), 1.0 ).normalized();
    }

    bool mergeSum( SurfacePatch& o, float gapSize )
    {
	SurfacePatch &p(*this);

	if( overlap( min-gapSize, max+gapSize, o.min, o.max ) )
	{
	    // use the weighted formulas for calculating 
	    // mean and var of occupied space distribution
	    // in the patch

	    p.plane.z += o.plane.z;
	    p.plane.zz += o.plane.zz;
	    p.plane.n += o.plane.n;
	    p.n += o.n;
	    p.normsq += o.normsq;
	    p.min = std::min( p.min, o.min );
	    p.max = std::max( p.max, o.max );

	    p.updateSum();

	    return true;
	}

	return false;
    }

    bool mergePlane( SurfacePatch& o, float gapSize )
    {
	SurfacePatch &p(*this);

	if( overlap( min-gapSize, max+gapSize, o.min, o.max ) )
	{
	    p.n += o.n;
	    p.normsq += o.normsq;
	    p.min = std::min( p.min, o.min );
	    p.max = std::max( p.max, o.max );

	    // sum the plane between the two
	    p.plane.update( o.plane );
	    p.updatePlane();
	    
	    return true;
	}
	
	return false;
    }

    bool mergeMLS( SurfacePatch& o, double thickness, double gapSize )
    {
	SurfacePatch &p(*this);
	const double delta_dev = sqrt( p.stdev * p.stdev + o.stdev * o.stdev );

	// see if the distance between the patches is small enough
	if( (p.mean - p.height - gapSize - delta_dev) < o.mean 
		&& (p.mean + gapSize + delta_dev) > (o.mean - o.height) )
	{
	    // if both patches are horizontal, we see if we can merge them
	    if( p.isHorizontal() && o.isHorizontal() ) 
	    {
		if( (p.mean - p.height - thickness - delta_dev) < o.mean && 
			(p.mean + thickness + delta_dev) > o.mean )
		{
		    kalman_update( p.mean, p.stdev, o.mean, o.stdev );
		}
		else
		{
		    // convert into a vertical patch element
		    //p.mean += p.stdev;
		    //p.height = 2 * p.stdev; 
		    p.setVertical();
		}
	    }

	    // if either of the patches is vertical, the result is also going
	    // to be vertical
	    if( !p.isHorizontal() || !o.isHorizontal())
	    {
		if( p.isHorizontal() && o.isVertical() )
		    p.setVertical();
		else if( p.isNegative() && o.isNegative() )
		    p.setNegative();
		else if( p.isNegative() || o.isNegative() )
		{
		    // in this case (one negative one non negative)
		    // its a bit hard to decide, since we have to remove
		    // something somewhere to make it compatible
		    //
		    // best is to decide on age (based on update_idx) 
		    // of the patch. Newer patches will be preferred

		    if( p.update_idx == o.update_idx )
			return false;

		    SurfacePatch &rp( p.update_idx < o.update_idx ? p : o );
		    SurfacePatch &ro( p.update_idx < o.update_idx ? o : p );

		    if( rp.update_idx < ro.update_idx )
		    {
			// the new patch fully encloses the old one, 
			// so will overwrite it
			if( ro.mean > rp.mean && ro.mean - ro.height < rp.mean - rp.height )
			{
			    p = ro;
			    return true; 
			} 

			// the other patch is occupied, so cut
			// the free patch accordingly
			if( ro.mean < rp.mean )
			    rp.height = rp.mean - ro.mean;
			else if( ro.mean - ro.height < rp.mean )
			{
			    double new_mean = ro.mean - ro.height;
			    rp.height -= rp.mean - new_mean;
			    rp.mean = new_mean;
			}

			// both patches can live 
			return false;
		    }
		}

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

	    return true;
	}

	return false;
    }

    bool merge( SurfacePatch& o, double thickness, double gapSize, MLSConfiguration::update_model updateModel )
    {
	bool merge = false;

	switch( updateModel )
	{
	    case MLSConfiguration::KALMAN:
		merge = mergeMLS( o, thickness, gapSize );
		break;

	    case MLSConfiguration::SUM:
		merge = mergeSum( o, gapSize );
		break;

	    case MLSConfiguration::SLOPE:
		merge = mergePlane( o, gapSize );
		break;

	    default:
		throw std::runtime_error("MLS update model not implemented.");
	}

	if( merge )
	{
	    update_idx = std::max( update_idx, o.update_idx );
	    // update cell color
	    setColor( (getColor() + o.getColor()) / 2.0 );

	    return true;
	}
	
	return false;
    }

    /** 
     * @brief get the weighting for the patch
     * This value will depend on the uncertainty with which it was applied. It
     * is the sum of the inverse square of the standard deviation.
     */ 
    float getWeight() const
    {
        return plane.n;
    }

    /**
     * @brief scale the weighting without affecting the uncertainty calculation
     * @param factor scale factor to apply
     */
    void scaleWeight( float factor )
    {
        plane.scale( factor );
        normsq *= pow( factor, 2 );
    }

    float getMean() const
    {
	return mean;
    }

    float getStdev() const
    {
	return stdev;
    }

    float getHeight() const
    {
	return height;
    }

    /**
     * Returns the amount of measurements
     * which are represented by this SufacePatch
     * */
    float getMeasurementCount() const
    {
        return n;
    }
public:
    /** The mean Z value. This always represents the top of the patch,
     * regardless whether the patch is horizontal or vertical
     */
    float mean;
    /** The standard deviation in Z */
    float stdev;
    /** For vertical patches, the height of the patch */
    float height;

    base::PlaneFitting<float> plane;
    float min, max;
    float n;
    float normsq;

    size_t update_idx;
    uint8_t color[3];

protected:
    /** Horizontal patches are just a mean and standard deviation.
     * Vertical patches also have a height, i.e. the patch is a vertical
     * block between z=(mean-height) and z
     */
    TYPE type;
};

}

#endif
