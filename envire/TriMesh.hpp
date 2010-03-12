#ifndef __TRIMESH_HPP__
#define __TRIMESH_HPP__

#include "Core.hpp"
#include "Pointcloud.hpp"
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <stdexcept>

class VectorHolder;

namespace envire {

    class VectorHolder
    {
	public:
	    virtual ~VectorHolder() {};
	    virtual void* getData() = 0;
	    template <typename T> std::vector<T>& get()
	    {
		return *static_cast<std::vector<T>*>( getData() );
	    }
    };

    template <typename T>
    class VectorH : public VectorHolder 
    {
	std::vector<T>* ptr;

	public:
	VectorH()
	{
	    ptr = new std::vector<T>();
	};

	~VectorH()
	{
	    delete ptr;
	};

	void* getData() 
	{
	    return ptr;
	}
    };

    class TriMesh : public Pointcloud 
    {
    public:
	typedef boost::tuple<int, int, int> triangle_t;
	typedef int vertex_attr;

	enum data_type
	{
	    VERTEX,
	    VERTEX_COLOR,
	    VERTEX_NORMAL,
	    VERTEX_ATTRIBUTES,
	    FACE
	};

	enum attr_flag
	{
	    SCAN_EDGE = 0x01 // vertex point is at the edge of a laserscan
	};

    public:
	/** vector of triangle_t, which are indeces into the points vector the
	 * number of trimeshes is independent of the number of points in the
	 * map
	 */
	std::vector<triangle_t> faces;

    public:
	static const std::string className;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	template <typename T>
	std::vector<T>& getData(data_type type)
	{
	    if( !hasData( type ) )
		data_map[type] = new VectorH<T>;

	    if( typeid(*data_map[type]) != typeid(VectorH<T>) )
	    {
		std::cerr 
		    << "type mismatch. type should be " 
		    << typeid(data_map[type]).name() 
		    << " but is " 
		    << typeid(VectorH<T>).name()
		    << std::endl;
		throw std::runtime_error("data type mismatch.");
	    }

	    return data_map[type]->get<T>();
	};

	bool hasData(data_type type);
	
	TriMesh();
	~TriMesh();

	TriMesh(Serialization& so);
	void serialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	const std::string& getClassName() const {return className;};

	TriMesh* clone();

	void calcVertexNormals( void );

    private:
	std::map<data_type, VectorHolder*> data_map;
    };
}

#endif
