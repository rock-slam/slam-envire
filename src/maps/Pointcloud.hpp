#ifndef __ENVIRE_POINTCLOUD_HPP__
#define __ENVIRE_POINTCLOUD_HPP__

#include <envire/Core.hpp>
#include <Eigen/Core>

namespace envire {
    class Pointcloud : public Map<3> 
    {
	ENVIRONMENT_ITEM( Pointcloud )

    public:
	typedef int vertex_attr;

	static const std::string VERTEX_COLOR;
	static const std::string VERTEX_NORMAL;
	static const std::string VERTEX_ATTRIBUTES;
	static const std::string VERTEX_VARIANCE;

	enum attr_flag
	{
	    SCAN_EDGE = 0x01 // vertex point is at the edge of a laserscan
	};

	/** definition of 3d points
	 */
	std::vector<Eigen::Vector3d> vertices;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static Pointcloud* importCsv(const std::string& file, FrameNode* fn);

	template <typename T>
	    std::vector<T>& getVertexData(const std::string& key)
	{
	    // TODO think about what to do with the sizing of the arrays
	    typedef std::vector<T> array_type;
	    array_type& data( getData<array_type>(key) );
	    return data;
	};

	void clear()
	{
	    vertices.clear();
	    if( hasData( VERTEX_COLOR ) ) getVertexData<Eigen::Vector3d>( VERTEX_COLOR ).clear();
	    if( hasData( VERTEX_NORMAL ) ) getVertexData<Eigen::Vector3d>( VERTEX_NORMAL ).clear();
	    if( hasData( VERTEX_ATTRIBUTES ) ) getVertexData<attr_flag>( VERTEX_ATTRIBUTES ).clear();
	    if( hasData( VERTEX_VARIANCE ) ) getVertexData<double>( VERTEX_VARIANCE ).clear();
	};

	Pointcloud();
	~Pointcloud();

	void copyFrom( Pointcloud* source, bool transform = true );

	Pointcloud(Serialization& so, bool handleMap = true);
	void serialize(Serialization& so);
	void serialize(Serialization& so, bool handleMap = true);
    void unserialize(Serialization& so, bool handleMap = true);

	bool writeText(const std::string& path);
	bool readText(const std::string& path);

	bool writePly(const std::string& path);
	bool readPly(const std::string& path);

	Extents getExtents() const;
    };
}

#endif
