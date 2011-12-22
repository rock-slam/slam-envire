#include "SurfaceReconstruction.hpp"

using namespace envire;
using namespace std;

#include <CGAL/trace.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>

const std::string SurfaceReconstruction::className = "envire::SurfaceReconstruction";

SurfaceReconstruction::SurfaceReconstruction()
{
}

SurfaceReconstruction::SurfaceReconstruction(Serialization& so)
{
    unserialize(so);
}

void SurfaceReconstruction::serialize(Serialization& so)
{
    Operator::serialize(so);
}

void SurfaceReconstruction::unserialize(Serialization& so)
{
    Operator::unserialize(so);
}

void SurfaceReconstruction::addInput( Pointcloud* mesh ) 
{
    if( env->getInputs(this).size() > 0 )
        throw std::runtime_error("SurfaceReconstruction can only have one input.");

    Operator::addInput(mesh);
}

void SurfaceReconstruction::addOutput( TriMesh* mesh )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("SurfaceReconstruction can only have one output.");

    Operator::addOutput(mesh);
}

// This stuff is necessary to have indexed points in a polyhedron, so that
// the vertices can be extracted with an index and converted back to the
// TriMesh format
template <class Refs, class T, class Point>
struct IndexedVertex : public CGAL::HalfedgeDS_vertex_base<Refs, T, Point>
{
    IndexedVertex() {};
    IndexedVertex( Point p ) : CGAL::HalfedgeDS_vertex_base<Refs,T,Point>( p ) {};
    size_t index;
};

struct IndexedItems : public CGAL::Polyhedron_items_3 
{
    template <class Refs, class Traits>
	struct Vertex_wrapper
	{
	    typedef typename Traits::Point_3 Point;
	    typedef IndexedVertex<Refs, CGAL::Tag_true, Point> Vertex;
	};
};

bool SurfaceReconstruction::updateAll() 
{
    // we'll convert the input pc to cgal first and convert back afterwards
    // probably not fully efficient, but easiest way for now

    Pointcloud* pc_in = static_cast<envire::Pointcloud*>(*env->getInputs(this).begin());
    assert(pc_in);
	
    TriMesh* mesh_out = static_cast<envire::TriMesh*>(*env->getOutputs(this).begin());
    assert(mesh_out);

    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point;
    typedef Kernel::Vector_3 Vector;
    typedef Kernel::FT FT;

    typedef boost::tuple<int, Point, Vector> IndexedPoint;
    std::vector<IndexedPoint> points;
    points.reserve( pc_in->vertices.size() );

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

    std::cout << "copied to cgal struct" << std::endl;
 
    // Reconstruction typedefs
    typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
    typedef CGAL::Surface_mesh_default_triangulation_3 STr;
    typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
    typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;
    typedef Kernel::Sphere_3 Sphere;

    // Poisson options
    FT sm_angle = 20.0; // Min triangle angle in degrees.
    FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
    FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.

    // Creates implicit function from the read points using the default solver (TAUCS).
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    Poisson_reconstruction_function function(
	    points.begin(), points.end(),
	    CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
	    CGAL::Nth_of_tuple_property_map<2,IndexedPoint>() );

    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.
    if ( ! function.compute_implicit_function() )
      return false;

    // Computes average spacing
    FT average_spacing = CGAL::compute_average_spacing(points.begin(), points.end(),
	    CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
	    6 );
    // Gets one point inside the implicit surface
    // and computes implicit function bounding sphere radius.
    Point inner_point = function.get_inner_point();
    Sphere bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());

    // Defines the implicit surface: requires defining a
    // conservative bounding sphere centered at inner point.
    FT sm_sphere_radius = 5.0 * radius;
    FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
	    Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
	    sm_dichotomy_error/sm_sphere_radius);

    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error

    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh

    

    typedef CGAL::Polyhedron_3<Kernel, IndexedItems> Polyhedron;

    // convert to polyhedron
    Polyhedron output_mesh;
    CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);

    // copy back into pointcloud structure
    size_t idx = 0;
    for( Polyhedron::Vertex_iterator it=output_mesh.vertices_begin();it!=output_mesh.vertices_end();it++)
    {
	Polyhedron::Point_3 &v(it->point());
	mesh_out->vertices.push_back( Eigen::Vector3d( v.x(), v.y(), v.z() ) );
	// tag the vertex with an index
	it->index = idx++;
    }

    std::cout << "copied points " << mesh_out->vertices.size() << std::endl;

    typedef Polyhedron::Facet_iterator Facet_iterator;
    typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

    for( Polyhedron::Facet_iterator it=output_mesh.facets_begin();it!=output_mesh.facets_end();it++)
    {
	TriMesh::triangle_t face;

	Halfedge_facet_circulator j = it->facet_begin();
        // Facets in polyhedral surfaces are at least triangles.
	if( CGAL::circulator_size(j) != 3 )
	    std::cout << CGAL::circulator_size(j) << ' ';

	int n = 0;
        do {
	    int vidx = j->vertex()->index; 
	    if( vidx >= mesh_out->vertices.size() )
		std::cout << "index " << vidx << " out of range!";

	    switch(n) {
		case(0): face.get<0>() = vidx; break;
		case(1): face.get<1>() = vidx; break;
		case(2): face.get<2>() = vidx; break;
		default: break;
	    }
	    n++;
        } while ( ++j != it->facet_begin());

	mesh_out->faces.push_back( face );
    }

    mesh_out->calcVertexNormals();

    std::cout << "copied faces " << mesh_out->faces.size() << std::endl;

}


