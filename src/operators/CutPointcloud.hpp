#ifndef ENVIRE_CUT_POINTCLOUD_H_
#define ENVIRE_CUT_POINTCLOUD_H_

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <Eigen/src/Geometry/AlignedBox.h>

namespace envire {
    
    struct ExclusionBox
    {
        Eigen::AlignedBox<double,3> box;
        bool exclude;
        ExclusionBox() : exclude(true) {};
        bool excludes() {return exclude;};
        bool includes() {return !exclude;};
    };
    
    class CutPointcloud: public Operator {

	ENVIRONMENT_ITEM( CutPointcloud )

    public:
	CutPointcloud();
	virtual ~CutPointcloud();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);
        
        void addBox(ExclusionBox* box);
        void removeBox(ExclusionBox* box);

	void addInput(Pointcloud* pc);
	void addOutput(Pointcloud* pc);
	bool updateAll();
        
    protected:
        bool isIncluded(const Eigen::Vector3d &vector);
        void copyVertexData(Pointcloud* source, Pointcloud* target, bool do_transform = true, bool filter = true);

    protected:
        std::list<ExclusionBox*> exclusion_boxes;
        boost::shared_ptr<Pointcloud> vertex_data_source;
    };
}

#endif
