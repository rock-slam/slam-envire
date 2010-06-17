#ifndef __ENVIRE_PLYFILE_HPP__
#define __ENVIRE_PLYFILE_HPP__

#include "TriMesh.hpp"
#include <tr1/functional>

#include <ply.hpp>

namespace envire
{
    class PlyFile
    {
    public:
	PlyFile( const std::string& filename );
	bool serialize( Pointcloud *pointcloud );
	bool unserialize( Pointcloud *pointcloud );

    private:
	std::string filename_;
	Pointcloud* pco_;

    private:
	void info_callback(const std::string& filename, std::size_t line_number, const std::string& message);
	void warning_callback(const std::string& filename, std::size_t line_number, const std::string& message);
	void error_callback(const std::string& filename, std::size_t line_number, const std::string& message);
	void magic_callback();
	void format_callback(ply::format_type format, const std::string& version);
	template <typename ScalarType> void scalar_property_callback(ScalarType scalar);
	template <typename ScalarType> void vector_property_callback(ScalarType scalar, std::vector<Eigen::Vector3d> *list, size_t idx, bool scale = false);
	template <typename ScalarType> std::tr1::function<void (ScalarType)> scalar_property_definition_callback(const std::string& element_name, const std::string& property_name);
	template <typename SizeType, typename ScalarType> void list_property_begin_callback(SizeType size);
	template <typename SizeType, typename ScalarType> void list_property_element_callback(ScalarType scalar);
	template <typename SizeType, typename ScalarType> void list_property_end_callback();
	template <typename SizeType, typename ScalarType> std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> > list_property_definition_callback(const std::string& element_name, const std::string& property_name);
	void comment_callback(const std::string& comment);
	void obj_info_callback(const std::string& obj_info);
	bool end_header_callback();

    };
}

#endif
