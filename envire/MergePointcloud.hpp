/*
 * MergePointcloud.h
 *
 *  Created on: 14.04.2010
 *      Author: planthaber
 */

#ifndef GLOBALPOINTCLOUD_H_
#define GLOBALPOINTCLOUD_H_

#include "Core.hpp"
#include "Pointcloud.hpp"

namespace envire {
    class MergePointcloud: public Operator {
    public:
	static const std::string className;

	MergePointcloud();
	MergePointcloud(Serialization& so);
	virtual ~MergePointcloud();

	void serialize(Serialization& so);

    public:
	void addInput(Pointcloud* pc);
	void addOutput(Pointcloud* globalpc);
	bool updateAll();
    };
}

#endif /* GLOBALPOINTCLOUD_H_ */
