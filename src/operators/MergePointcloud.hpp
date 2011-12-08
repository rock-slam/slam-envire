/*
 * MergePointcloud.h
 *
 *  Created on: 14.04.2010
 *      Author: planthaber
 */

#ifndef GLOBALPOINTCLOUD_H_
#define GLOBALPOINTCLOUD_H_

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>

namespace envire {
    class MergePointcloud: public Operator {

	ENVIRONMENT_ITEM( MergePointcloud )

    public:
	MergePointcloud();
	MergePointcloud(Serialization& so);
	virtual ~MergePointcloud();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

    public:
	void addInput(Pointcloud* pc);
	void addOutput(Pointcloud* globalpc);
	bool updateAll();
    };
}

#endif /* GLOBALPOINTCLOUD_H_ */
