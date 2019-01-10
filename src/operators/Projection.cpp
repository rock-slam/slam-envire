#include "Projection.hpp"
#include <stdexcept>
#include <stdint.h>
#include <limits>
#include "boost/multi_array.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <Eigen/LU>

using namespace envire;
using namespace std;

ENVIRONMENT_ITEM_DEF( Projection )

Projection::Projection()
{
}

void Projection::serialize(Serialization& so)
{
    Operator::serialize(so);
}

void Projection::unserialize(Serialization& so)
{
    Operator::unserialize(so);
}

void Projection::addInput( Pointcloud* mesh ) 
{
    Operator::addInput(mesh);
}

void Projection::addOutput( ElevationGrid* grid )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("Projection can only have one output.");

    Operator::addOutput(grid);
}

bool Projection::updateAll() 
{
    updateElevationMap();
    interpolateMap(ElevationGrid::ELEVATION_MAX);
    //updateTraversibilityMap();

    return true;
}

bool Projection::updateElevationMap()
{
    // TODO add checking of connections
    ElevationGrid* grid = static_cast<envire::ElevationGrid*>(*env->getOutputs(this).begin());

    ElevationGrid::ArrayType& elv_min(grid->getGridData(ElevationGrid::ELEVATION_MIN));
    ElevationGrid::ArrayType& elv_max(grid->getGridData(ElevationGrid::ELEVATION_MAX));

    // fill the elevation map
    std::fill(elv_min.data(), elv_min.data() + elv_min.num_elements(), std::numeric_limits<double>::infinity());
    std::fill(elv_max.data(), elv_max.data() + elv_max.num_elements(), -std::numeric_limits<double>::infinity());

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	Pointcloud* mesh = dynamic_cast<envire::Pointcloud*>(*it);

	FrameNode::TransformType C_m2g = env->relativeTransform( mesh->getFrameNode(), grid->getFrameNode() );

	std::vector<Eigen::Vector3d>& points(mesh->vertices);
	
	for(size_t i=0;i<points.size();i++)
	{
	    Eigen::Vector3d p = env->getRootNode()->getTransform() * C_m2g * points[i];

	    size_t x, y;
	    if( grid->toGrid( p.x(), p.y(), x, y ) )
	    {
		elv_max[y][x] = std::max( elv_max[y][x], p.z() );
		elv_min[y][x] = std::min( elv_min[y][x], p.z() );
	    }
	}
    }

    return true;
}

bool Projection::interpolateMap(const std::string& type)
{
    // TODO add checking of connections
    ElevationGrid* grid = static_cast<envire::ElevationGrid*>(*env->getOutputs(this).begin());

    if( !grid->hasData( type ) )
	return false;

    ElevationGrid::ArrayType& data(grid->getGridData(type));

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Projection_traits_xy_3<K>  Gt;
    typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;

    typedef K::Point_3 Point;

    Delaunay dt;

    size_t width = grid->getCellSizeY();
    size_t height = grid->getCellSizeX();
    
    size_t min_width = grid->getCellSizeY() - 1;
    size_t max_width = 0;
    size_t min_height = grid->getCellSizeX() - 1;
    size_t max_height = 0;

    for(size_t x=0;x<width;x++)
    {
	for(size_t y=0;y<height;y++)
	{
	    if( fabs( data[x][y] ) != std::numeric_limits<double>::infinity() )
	    {
		Point p(x,y,data[x][y] );
		dt.insert( p );
                
                min_width = std::min(min_width, x);
                max_width = std::max(max_width, x);
                min_height = std::min(min_height, y);
                max_height = std::max(max_height, y);
	    }
	}
    }

    for(size_t x=min_width;x<max_width;x++)
    {
	for(size_t y=min_height;y<max_height;y++)
	{
	    if( fabs( data[x][y] ) == std::numeric_limits<double>::infinity() )
	    {
		// no data point in grid, so value needs to be interpolated

		// Solve linear equation system to find plane that is spanned by
		// the three points
		Eigen::Matrix3d A;
		Eigen::Vector3d b;
		
		Delaunay::Face_handle face = dt.locate( Point(x,y,0) );
		if( face == NULL || face->vertex(0) == NULL || face->vertex(1) == NULL || face->vertex(2) == NULL)
		    continue;

		for(int i=0;i<3;i++)
		{
		    Point &p(face->vertex(i)->point());
		    A.block<1,3>(i,0) = Eigen::Vector3d(p.x(), p.y(), 1);
		    b(i) = p.z();
		}

		// evaluate the point at x, y
		data[x][y] = Eigen::Vector3d(x,y,1).dot( A.inverse() * b );
	    }
	}
    }

    return true;
}

// TODO add this to a new operator
/*
bool Projection::updateTraversibilityMap()
{
    // TODO add checking of connections
    Grid* grid = static_cast<envire::Grid*>(*env->getOutputs(this).begin());

    if( !grid->hasData(Grid::ELEVATION_MIN) || !grid->hasData(Grid::ELEVATION_MAX) )
    {
	std::cout << "needs DEM map to calculate traversibility map." << std::endl;
	return false;
    }

    boost::multi_array<double,2>& elv_min(grid->getGridData<double>(Grid::ELEVATION_MIN));
    boost::multi_array<double,2>& elv_max(grid->getGridData<double>(Grid::ELEVATION_MAX));

    // compute the traversability map now...
    // this should most probably be done somewhere else, but is here for convenience now.
    boost::multi_array<uint8_t,2>& trav(grid->getGridData<uint8_t>(Grid::TRAVERSABILITY));
    std::fill(trav.data(), trav.data() + trav.num_elements(), 255);

    size_t width = elv_min.shape()[0]; 
    size_t height = elv_min.shape()[1]; 

    double scalex = grid->getScaleX();
    double scaley = grid->getScaleY();
    
    for(int x=1;x<(width-1);x++)
    {
	for(int y=1;y<(height-1);y++)
	{
	    double max_grad = -std::numeric_limits<double>::infinity();

	    for(int i=0;i<3;i++)
	    {
		// get the 4 neighbours
		int m = (i<2)*(i*2-1);
		int n = (i>=2)*((i-2)*2-1);
		double scale = (i<2)?scalex:scaley;

		double gradient =
		   std::max( 
			  std::abs(elv_min[x][y] - elv_max[x+m][y+n])/scale,
			  std::abs(elv_max[x][y] - elv_min[x+m][y+n])/scale );

		max_grad = std::max( max_grad, gradient );
	    }

	    double angle = atan( max_grad );

	    trav[x][y] = ((uint8_t)(angle/(M_PI/2.0)*6.0));
	}
    }

    return true;
}
*/

