#ifndef _MLS_TO_POINTCLOUD_HPP_
#define _MLS_TO_POINTCLOUD_HPP_

#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Pointcloud.hpp>


namespace envire 
{
	/**
	 * This operator generates a Pointcloud from a MLSGrid
	 * 
	 * It can only have one input and one output
	 */
	
    class MLSToPointCloud : public Operator
    {
	ENVIRONMENT_ITEM( MLSToPointCloud )
	
	private:
		using Operator::addInput;
		using Operator::addOutput;
		
	public:
		MLSToPointCloud();
		virtual ~MLSToPointCloud();
		
		void setInput(MLSGrid* mls_grid);
		void setOutput(Pointcloud* pointcloud);
		
		bool updateAll();
    };
}


#endif // _MLS_TO_POINTCLOUD_HPP_