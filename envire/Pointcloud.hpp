#ifndef __ENVIRE_POINTCLOUD_HPP__
#define __ENVIRE_POINTCLOUD_HPP__

#include "Core.hpp"
#include <Eigen/Core>

namespace envire {
    class Pointcloud : public CartesianMap 
    {
    public:
	/** definition of 3d points
	 */
	std::vector<Eigen::Vector3d> vertices;

    public:
	static const std::string className;

	static Pointcloud* importCsv(const std::string& file, FrameNode* fn);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Pointcloud();
	~Pointcloud();

	Pointcloud(Serialization& so, bool handleMap = true);
	void serialize(Serialization& so, bool handleMap = true);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	const std::string& getClassName() const {return className;};

	Pointcloud* clone();
    };
}

#endif
