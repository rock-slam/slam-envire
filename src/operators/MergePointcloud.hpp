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
	virtual ~MergePointcloud();

	void serialize(Serialization& so);
        void unserialize(Serialization& so);

	/** @brief set to true to clear output
	 * before each merge operation
	 */
	void setClearOutput( bool clear );

    public:
	void addInput(Pointcloud* pc);
	void addOutput(Pointcloud* globalpc);
	bool updateAll();

    protected:
	bool m_clearOutput;
    };
}

#endif /* GLOBALPOINTCLOUD_H_ */
