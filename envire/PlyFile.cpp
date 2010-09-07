#include "PlyFile.hpp"
#include <fstream>
#include <tr1/functional>

using namespace envire;
using namespace std::tr1::placeholders;

template <typename ScalarType>
void PlyFile::scalar_property_callback(ScalarType scalar)
{
}

template <typename ScalarType>
void PlyFile::vector_property_callback(ScalarType scalar, std::vector<Eigen::Vector3d> *list, size_t idx, bool scale)
{
    static Eigen::Vector3d vector;
    if( scale )
	vector[idx] = scalar/255.0;
    else
	vector[idx] = scalar;

    if( idx == 2 )
    {
	list->push_back( vector );
    }
}

template <typename ScalarType>
std::tr1::function<void (ScalarType)> PlyFile::scalar_property_definition_callback(const std::string& element_name, const std::string& property_name)
{
    if( element_name == "vertex" )
    {
	if( property_name == "x" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &pco_->vertices, 0, false);
	if( property_name == "y" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &pco_->vertices, 1, false);
	if( property_name == "z" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &pco_->vertices, 2, false);
    }

    if( element_name == "normal" )
    {
	std::vector<Eigen::Vector3d> &normals( pco_->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL ) );
	
	if( property_name == "x" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &normals, 0, false);
	if( property_name == "y" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &normals, 1, false);
	if( property_name == "z" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &normals, 2, false);
    }

    if( element_name == "color" )
    {
	std::vector<Eigen::Vector3d> &colors( pco_->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR ) );
	
	if( property_name == "red" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &colors, 0, true);
	if( property_name == "green" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &colors, 1, true);
	if( property_name == "blue" )
	    return std::tr1::bind(&PlyFile::vector_property_callback<ScalarType>, this, _1, &colors, 2, true);
    }

    return std::tr1::bind(&PlyFile::scalar_property_callback<ScalarType>, this, _1);
}

int triangle_idx__;
TriMesh::triangle_t triangle__;

template <typename SizeType, typename ScalarType>
void PlyFile::list_property_begin_callback(SizeType size)
{
    if( size != 3 )
	std::cerr << "no support for faces with edgecount different to 3 (is " << (int)size << ")." << std::endl;

    triangle_idx__ = 0;
}

template <typename SizeType, typename ScalarType>
void PlyFile::list_property_element_callback(ScalarType scalar)
{
    if( scalar >= pco_->vertices.size() )
	std::cerr << "vertex_index " << scalar << " is out of range!" << std::endl;

    switch( triangle_idx__ )
    {
	case(0): triangle__.get<0>() = scalar; break;
	case(1): triangle__.get<1>() = scalar; break;
	case(2): triangle__.get<2>() = scalar; break;
	default: break;
    }

    triangle_idx__++;
}

template <typename SizeType, typename ScalarType>
void PlyFile::list_property_end_callback()
{
    tmo_->faces.push_back( triangle__ );
}

template <typename DummyType>
void PlyFile::list_property_dummy_callback(DummyType dummy)
{
}

template <typename SizeType, typename ScalarType>
std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> > PlyFile::list_property_definition_callback(const std::string& element_name, const std::string& property_name)
{
    if( element_name == "face" && property_name == "vertex_index" && tmo_ )
    {
	return std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> >(
		std::tr1::bind(&PlyFile::list_property_begin_callback<SizeType, ScalarType>, this, _1),
		std::tr1::bind(&PlyFile::list_property_element_callback<SizeType, ScalarType>, this, _1),
		std::tr1::bind(&PlyFile::list_property_end_callback<SizeType, ScalarType>, this)
		);
    }
    else
    {
	return std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> >(
		std::tr1::bind(&PlyFile::list_property_dummy_callback<SizeType>, this, _1),
		std::tr1::bind(&PlyFile::list_property_dummy_callback<ScalarType>, this, _1),
		std::tr1::bind(&PlyFile::list_property_dummy_callback<ScalarType>, this, 0)
		);

	std::cerr << "no support for " << element_name << " " << property_name << std::endl;
    }
}

void PlyFile::info_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
    std::cerr << filename << ":" << line_number << ": " << "info: " << message << std::endl;
}

void PlyFile::warning_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
    std::cerr << filename << ":" << line_number << ": " << "warning: " << message << std::endl;
}

void PlyFile::error_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
    std::cerr << filename << ":" << line_number << ": " << "error: " << message << std::endl;
}

void PlyFile::magic_callback()
{
}

void PlyFile::format_callback(ply::format_type format, const std::string& version)
{
}

void PlyFile::comment_callback(const std::string& comment)
{
}

void PlyFile::obj_info_callback(const std::string& obj_info)
{
}

bool PlyFile::end_header_callback()
{
    return true;
}

PlyFile::PlyFile( const std::string& filename )
    : filename_( filename ), pco_(NULL), tmo_(NULL)
{
}

bool PlyFile::serialize( Pointcloud *pointcloud )
{
    std::ofstream data(filename_.c_str());
    if( data.fail() )  
    {
	std::cerr << "Could not open file '" + filename_ + "' for writing." << std::endl;
	return false;
    }

    const std::string version = "1.0";

    data << "ply" << "\n";
    data << "format ";
    if( ply::host_byte_order == ply::little_endian_byte_order )
    {
	data << "binary_little_endian";
    }	
    else 
    {
	data << "binary_big_endian";
    }
    data << " " << version << "\n";
    data << "comment generated by envire" << "\n";

    data << "element vertex " << pointcloud->vertices.size() <<  "\n";
    data << "property double x\n";
    data << "property double y\n";
    data << "property double z\n";

    if( pointcloud->hasData( Pointcloud::VERTEX_NORMAL ) )
    {
	data << "element normal " << pointcloud->vertices.size() <<  "\n";
	data << "property double x\n";
	data << "property double y\n";
	data << "property double z\n";
    }

    if( pointcloud->hasData( Pointcloud::VERTEX_COLOR ) )
    {
	data << "element color " << pointcloud->vertices.size() <<  "\n";
	data << "property uchar red\n";
	data << "property uchar green\n";
	data << "property uchar blue\n";
    }

    // see if we can upcast to a trimesh
    TriMesh* trimesh = dynamic_cast<TriMesh*>(pointcloud);
    if( trimesh )
    {
	data << "element face " << trimesh->faces.size() << "\n";
	data << "property list uchar int vertex_index\n";
    }
    
    data << "end_header\n";

    // write the binary raw data now
    for(size_t i=0;i<pointcloud->vertices.size();i++)
    {
	Eigen::Vector3d &vertex( pointcloud->vertices[i] );
	data.write( reinterpret_cast<char*>(&vertex.x()), sizeof(double) );
	data.write( reinterpret_cast<char*>(&vertex.y()), sizeof(double) );
	data.write( reinterpret_cast<char*>(&vertex.z()), sizeof(double) );
    }

    if( pointcloud->hasData( Pointcloud::VERTEX_NORMAL ) )
    {
	std::vector<Eigen::Vector3d> &normals( pointcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_NORMAL ) );
	for(size_t i=0;i<normals.size();i++)
	{
	    Eigen::Vector3d &normal( normals[i] );
	    data.write( reinterpret_cast<char*>(&normal.x()), sizeof(double) );
	    data.write( reinterpret_cast<char*>(&normal.y()), sizeof(double) );
	    data.write( reinterpret_cast<char*>(&normal.z()), sizeof(double) );
	}
    }

    if( pointcloud->hasData( Pointcloud::VERTEX_COLOR ) )
    {
	std::vector<Eigen::Vector3d> &colors( pointcloud->getVertexData<Eigen::Vector3d>( Pointcloud::VERTEX_COLOR ) );
	for(size_t i=0;i<colors.size();i++)
	{
	    unsigned char r = colors[i].x()*255; 
	    unsigned char g = colors[i].y()*255; 
	    unsigned char b = colors[i].z()*255; 
	    data.write( reinterpret_cast<char*>(&r), sizeof(unsigned char) );
	    data.write( reinterpret_cast<char*>(&g), sizeof(unsigned char) );
	    data.write( reinterpret_cast<char*>(&b), sizeof(unsigned char) );
	}
    }

    if( trimesh )
    {
	for(size_t i=0;i<trimesh->faces.size();i++)
	{
	    unsigned char trinum = 3;
	    TriMesh::triangle_t &tri( trimesh->faces[i] );
	    int32_t e1 = tri.get<0>();
	    int32_t e2 = tri.get<1>();
	    int32_t e3 = tri.get<2>();
	    data.write( reinterpret_cast<char*>(&trinum), sizeof(unsigned char) );
	    data.write( reinterpret_cast<char*>(&e1), sizeof( int32_t ) );
	    data.write( reinterpret_cast<char*>(&e2), sizeof( int32_t ) );
	    data.write( reinterpret_cast<char*>(&e3), sizeof( int32_t ) );
	}
    }

    return true;
}

bool PlyFile::unserialize( Pointcloud *pointcloud )
{
    std::ifstream data(filename_.c_str());
    if( data.fail() )  
    {
	std::cerr << "Could not open file '" + filename_ + "'." << std::endl;
	return false;
    }

    pco_ = pointcloud;
    tmo_ = dynamic_cast<TriMesh*>(pointcloud);
    
    ply::ply_parser::flags_type ply_parser_flags = 0;
    ply::ply_parser ply_parser(ply_parser_flags);

    std::string ifilename;

    ply_parser.info_callback(std::tr1::bind(&PlyFile::info_callback, this, std::tr1::ref(ifilename), _1, _2));
    ply_parser.warning_callback(std::tr1::bind(&PlyFile::warning_callback, this, std::tr1::ref(ifilename), _1, _2));
    ply_parser.error_callback(std::tr1::bind(&PlyFile::error_callback, this, std::tr1::ref(ifilename), _1, _2));

    ply_parser.magic_callback(std::tr1::bind(&PlyFile::magic_callback, this));
    ply_parser.format_callback(std::tr1::bind(&PlyFile::format_callback, this, _1, _2));

    ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;

    ply::at<ply::int8>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::int8>, this, _1, _2);
    ply::at<ply::int16>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::int16>, this, _1, _2);
    ply::at<ply::int32>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::int32>, this, _1, _2);
    ply::at<ply::uint8>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::uint8>, this, _1, _2);
    ply::at<ply::uint16>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::uint16>, this, _1, _2);
    ply::at<ply::uint32>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::uint32>, this, _1, _2);
    ply::at<ply::float32>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::float32>, this, _1, _2);
    ply::at<ply::float64>(scalar_property_definition_callbacks) = std::tr1::bind(&PlyFile::scalar_property_definition_callback<ply::float64>, this, _1, _2);

    ply_parser.scalar_property_definition_callbacks(scalar_property_definition_callbacks);

    ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;

    ply::at<ply::uint8, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::int8>, this, _1, _2);
    ply::at<ply::uint8, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::int16>, this, _1, _2);
    ply::at<ply::uint8, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::int32>, this, _1, _2);
    ply::at<ply::uint8, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::uint8>, this, _1, _2);
    ply::at<ply::uint8, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::uint16>, this, _1, _2);
    ply::at<ply::uint8, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::uint32>, this, _1, _2);
    ply::at<ply::uint8, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::float32>, this, _1, _2);
    ply::at<ply::uint8, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint8, ply::float64>, this, _1, _2);

    ply::at<ply::uint16, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::int8>, this, _1, _2);
    ply::at<ply::uint16, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::int16>, this, _1, _2);
    ply::at<ply::uint16, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::int32>, this, _1, _2);
    ply::at<ply::uint16, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::uint8>, this, _1, _2);
    ply::at<ply::uint16, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::uint16>, this, _1, _2);
    ply::at<ply::uint16, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::uint32>, this, _1, _2);
    ply::at<ply::uint16, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::float32>, this, _1, _2);
    ply::at<ply::uint16, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint16, ply::float64>, this, _1, _2);

    ply::at<ply::uint32, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::int8>, this, _1, _2);
    ply::at<ply::uint32, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::int16>, this, _1, _2);
    ply::at<ply::uint32, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::int32>, this, _1, _2);
    ply::at<ply::uint32, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::uint8>, this, _1, _2);
    ply::at<ply::uint32, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::uint16>, this, _1, _2);
    ply::at<ply::uint32, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::uint32>, this, _1, _2);
    ply::at<ply::uint32, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::float32>, this, _1, _2);
    ply::at<ply::uint32, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&PlyFile::list_property_definition_callback<ply::uint32, ply::float64>, this, _1, _2);

    ply_parser.list_property_definition_callbacks(list_property_definition_callbacks);

    ply_parser.comment_callback(std::tr1::bind(&PlyFile::comment_callback, this, _1));
    ply_parser.obj_info_callback(std::tr1::bind(&PlyFile::obj_info_callback, this, _1));
    ply_parser.end_header_callback(std::tr1::bind(&PlyFile::end_header_callback, this));

    ply_parser.parse(data);

    return true;
}

