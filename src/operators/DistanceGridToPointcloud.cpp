#include "DistanceGridToPointcloud.hpp"

#include <envire/maps/Grids.hpp>
#include <envire/maps/Pointcloud.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( DistanceGridToPointcloud )

bool DistanceGridToPointcloud::updateAll()
{
    //if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
    //    throw std::runtime_error("DistanceGridToPointcloud needs to have exactly 1 input and 1 output for now.");
    //
    //DistanceGrid const& distanceGrid = *env->getInput<DistanceGrid*>(this);
    //Pointcloud& pointcloud = *env->getOutput< Pointcloud* >(this);

    // get the inputs, we need a distance grid, and we can have an optional ImageGrid
    std::list<Layer*> inputs = env->getInputs(this);
    ImageRGB24 *image = NULL;
    ImageRGB24::ArrayType *ir = NULL, *ig = NULL, *ib = NULL;
    
    DistanceGrid* dist = NULL;
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	if( dynamic_cast<ImageRGB24*>( *it ) )
	{
	    image = dynamic_cast<ImageRGB24*>( *it );
	    ir = &image->getGridData( ImageRGB24::R );
	    ig = &image->getGridData( ImageRGB24::G );
	    ib = &image->getGridData( ImageRGB24::B );
	}

	if( dynamic_cast<DistanceGrid*>( *it ) )
	    dist = dynamic_cast<DistanceGrid*>( *it );
    }

    DistanceGrid const& distanceGrid = *dist;
    Pointcloud& pointcloud = *env->getOutput< Pointcloud* >(this);

    // get relative transform from grid frame to pointcloud frame
    Transform t = distanceGrid.getFrameNode()->relativeTransform( pointcloud.getFrameNode() );
    bool needsTransform = !t.isApprox( Transform( Transform::Identity() ) );

    // the distance grid is a projection of the original pointcloud.
    // in order recover the pointcloud, we need to reverse the projection

    DistanceGrid::ArrayType const& distance = distanceGrid.getGridData( DistanceGrid::DISTANCE );
    std::vector<double>& uncertainty(pointcloud.getVertexData<double>(Pointcloud::VERTEX_VARIANCE));
    std::vector<Eigen::Vector3d> *color = NULL;
    if( image )
	color = &pointcloud.getVertexData<Eigen::Vector3d>(Pointcloud::VERTEX_COLOR);

    // clear target
    pointcloud.clear();
    
    typedef DistanceGrid::Position Position;
    for(size_t x=0; x<distanceGrid.getWidth(); x++)
    {
	for(size_t y=0; y<distanceGrid.getHeight(); y++)
	{
	    // only process vector if distance value is not NaN or inf
	    const float d = distance[y][x];
	    if( boost::math::isnormal( d ) && d < maxDistance ) 
	    {
		// construct (p_x,p_y,1.0) vector
		Eigen::Vector3d r;
		r << distanceGrid.fromGrid( Position( x, y ) ), 1.0;

		// scale the vector
		r *= d;

		// only transform to target if needed
		if( needsTransform )
		    r = t * r;

		// add point to target pointcloud
		pointcloud.vertices.push_back( r );

		// add uncertainty
		uncertainty.push_back( d * uncertaintyFactor );

		// add texture color information if image is there
		if( image )
		{
		    // get color value from image
		    const double f = 1.0/255.0;
		    Eigen::Vector3d c( (*ir)[y][x] * f, (*ig)[y][x] * f, (*ib)[y][x] * f );
		    color->push_back( c );
		}
	    }
	}
    }

    pointcloud.itemModified();
    return true;
}
