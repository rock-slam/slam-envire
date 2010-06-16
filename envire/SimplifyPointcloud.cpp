#include "SimplifyPointcloud.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/property_map.h>

using namespace envire;
using namespace std;

const std::string SimplifyPointcloud::className = "envire::SimplifyPointcloud";

SimplifyPointcloud::SimplifyPointcloud()
{
}

SimplifyPointcloud::SimplifyPointcloud(Serialization& so)
    : Operator(so)
{
    so.setClassName(className);
}

void SimplifyPointcloud::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.setClassName(className);
}


void SimplifyPointcloud::addInput( Pointcloud* mesh ) 
{
    if( env->getInputs(this).size() > 0 )
        throw std::runtime_error("SimplifyPointcloud can only have one input.");

    Operator::addInput(mesh);
}

void SimplifyPointcloud::addOutput( Pointcloud* grid )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("SimplifyPointcloud can only have one output.");

    Operator::addOutput(grid);
}

bool SimplifyPointcloud::updateAll() 
{
    // we'll convert the input pc to cgal first and convert back afterwards
    // probably not fully efficient, but easiest way for now

    Pointcloud* pc_in = static_cast<envire::Pointcloud*>(*env->getInputs(this).begin());
    assert(pc_in);
	
    Pointcloud* pc_out = static_cast<envire::Pointcloud*>(*env->getOutputs(this).begin());
    assert(pc_out);

    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point;
    typedef Kernel::Vector_3 Vector;
    typedef Kernel::FT FT;

    typedef boost::tuple<int, Point, Vector> IndexedPoint;
    std::vector<IndexedPoint> points;

    // copy to CGAL structure
    if( pc_in->hasData( Pointcloud::VERTEX_NORMAL ) )
    {
	std::vector<Eigen::Vector3d> &normals( pc_in->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL ) );
	for( int i=0;i<pc_in->vertices.size();i++ )
	{
	    Eigen::Vector3d &vertex(pc_in->vertices[i]);
	    Eigen::Vector3d &normal(normals[i]);
	    points.push_back( boost::make_tuple( i, Point(vertex.x(), vertex.y(), vertex.z()), Vector(normal.x(), normal.y(), normal.z()) ) );
	}
    }
    else
    {
	for( int i=0;i<pc_in->vertices.size();i++ )
	{
	    Eigen::Vector3d &vertex(pc_in->vertices[i]);
	    points.push_back( boost::make_tuple( i, Point(vertex.x(), vertex.y(), vertex.z()), Vector() ) );
	}
    }

    // compute average spacing
    const unsigned int as_neighbors = 6; // 1 ring
    FT average_spacing = CGAL::compute_average_spacing(
	    points.begin(), points.end(),
	    CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
	    as_neighbors);
 
    // simplification of point set
    //double cell_size = 0.001;
    //points.erase(CGAL::grid_simplify_point_set(points.begin(), points.end(), CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(), cell_size), points.end() );
    double percentage = 0.5;
    points.erase(CGAL::random_simplify_point_set(points.begin(), points.end(), CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(), percentage), points.end() );

    // Removes outliers using erase-remove idiom.
    // The Dereference_property_map property map can be omitted here as it is the default value.
    const double removed_percentage = 5.0; // percentage of points to remove
    const int or_neighbors = 24; // considers 24 nearest neighbor points
    points.erase(CGAL::remove_outliers(points.begin(), points.end(),
		CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
		or_neighbors, removed_percentage), 
	    points.end());


    const int sm_neighbors = 24; // considers 24 nearest neighbor points
    CGAL::jet_smooth_point_set(points.begin(), points.end(), 
	    CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
	    sm_neighbors);

}


