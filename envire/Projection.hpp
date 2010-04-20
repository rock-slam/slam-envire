#ifndef __ENVIRE_PROJECTION_HPP__
#define __ENVIRE_PROJECTION_HPP__

#include "Core.hpp" 
#include "TriMesh.hpp" 
#include "Grid.hpp" 

#include <stdexcept>
#include <stdint.h>
#include <limits>
#include "boost/multi_array.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_euclidean_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <Eigen/LU>
#include <Eigen/Core>

namespace envire {
    template <typename T>
    class Projection : public Operator
    {

    public:
	static const std::string className;

	Projection();

	Projection(Serialization& so);
	void serialize(Serialization& so);

	const std::string& getClassName() const {return className;};

	void addInput( TriMesh* mesh ); 
	void addOutput( Grid<T>* grid ); 

	bool updateAll();
	bool updateElevationMap();
	bool interpolateMap(const std::string& type);
	bool updateTraversibilityMap();
    };
   
    template <typename T> const std::string Projection<T>::className = "envire::Projection_"+ std::string(typeid(T).name());
    template <typename T> Projection<T>::Projection()
    {
    }

    template <typename T> Projection<T>::Projection(Serialization& so)
	: Operator(so)
    {
	so.setClassName(className);
    }


    template <typename T> void Projection<T>::serialize(Serialization& so)
    {
	Operator::serialize(so);
	so.setClassName(className);
    }


    template <typename T> void Projection<T>::addInput( TriMesh* mesh ) 
    {
	Operator::addInput(mesh);
    }

    template <typename T> void Projection<T>::addOutput( Grid<T>* grid )
    {
	if( env->getOutputs(this).size() > 0 )
	    throw std::runtime_error("Projection can only have one output.");

	Operator::addOutput(grid);
    }

    template <typename T> bool Projection<T>::updateAll() 
    {
	updateElevationMap();
	//updateTraversibilityMap();
	//interpolateMap("");
    }
    
    template <typename T> bool Projection<T>::updateElevationMap()
    {
	// TODO add checking of connections
	std::cout<< "Projection Update is called" << std::endl;
	Grid<T>* grid = static_cast<envire::Grid<T>*>(*env->getOutputs(this).begin());
	typename Grid<T>::ArrayType& elv(grid->getGridData());
	
	// fill the elevation map with default values
	std::fill(elv.data(), elv.data() + elv.num_elements(), -std::numeric_limits<T>::infinity());

	std::list<Layer*> inputs = env->getInputs(this);
	for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
	{
	    TriMesh* mesh = dynamic_cast<envire::TriMesh*>(*it);
	    FrameNode::TransformType C_m2g = env->relativeTransform( mesh->getFrameNode(), grid->getFrameNode() );
	    std::vector<Eigen::Vector3d>& points(mesh->vertices);
	    for(int i=0;i<points.size();i++)
	    {
		Eigen::Vector3d p = env->getRootNode()->getTransform() * C_m2g * points[i];
		size_t x, y;
		if( grid->toGrid( p.x(), p.y(), x, y ) )
		{
		    elv[y][x] = std::max( elv[y][x], p.z() );
		} 
	    } 
	}
	return true; 
    }

    template <typename T> bool Projection<T>::interpolateMap(const std::string& type)
    {
	// TODO add checking of connections
	Grid<T>* grid = static_cast<envire::Grid<T>*>(*env->getOutputs(this).begin());
	typename Grid<T>::ArrayType& data(grid->getGridData());

	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Triangulation_euclidean_traits_xy_3<K>  Gt;
	typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;

	typedef K::Point_3 Point;

	Delaunay dt;

	size_t width = data.getWidth(); 
	size_t height = data.getHeight(); 
	
	for(int x=0;x<width;x++)
	{
	    for(int y=0;y<height;y++)
	    {
		if( fabs( data[y][x] ) != std::numeric_limits<double>::infinity() )
		{
		    Point p(x,y,data[y][x] );
		    dt.insert( p );
		}
	    }
	}

	for(int x=0;x<width;x++)
	{
	    for(int y=0;y<height;y++)
	    {
		if( fabs( data[y][x] ) == std::numeric_limits<double>::infinity() )
		{
		    // no data point in grid, so value needs to be interpolated

		    // Solve linear equation system to find plane that is spanned by
		    // the three points
		    Eigen::Matrix3d A;
		    Eigen::Vector3d b;
		    
		    Delaunay::Face_handle face = dt.locate( Point(x,y,0) );
		    for(int i=0;i<3;i++)
		    {
			Point &p(face->vertex(i)->point());
			A.block<1,3>(i,0) = Eigen::Vector3d(p.x(), p.y(), 1);
			b(i) = p.z();
		    }

		    // evaluate the point at x, y
		    data[y][x] = Eigen::Vector3d(x,y,1).dot( A.inverse() * b );
		}
	    }
	}
	return true;
    }

    template <typename T> bool Projection<T>::updateTraversibilityMap()
    {
	// TODO add checking of connections
	/*Grid* grid = static_cast<envire::Grid*>(*env->getOutputs(this).begin());

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
	}*/
	return true;
    }
}

#endif
