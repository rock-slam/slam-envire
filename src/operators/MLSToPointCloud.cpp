#include "MLSToPointCloud.hpp"


using namespace envire;

ENVIRONMENT_ITEM_DEF( MLSToPointCloud )

MLSToPointCloud::MLSToPointCloud(): Operator(1, 1)
{

}

MLSToPointCloud::~MLSToPointCloud()
{

}

void MLSToPointCloud::setInput(MLSGrid* mls_grid)
{
    Operator::setInput(mls_grid);
}

void MLSToPointCloud::setOutput(Pointcloud* pointcloud)
{
    Operator::setOutput(pointcloud);
}


bool MLSToPointCloud::updateAll()
{
    
    Pointcloud* pointcloud = dynamic_cast<Pointcloud*>(env->getOutput<Pointcloud*>(this));    
    MLSGrid* mls_grid = dynamic_cast<MLSGrid*>(env->getInput<MLSGrid*>(this));
    
    pointcloud->clear();

    float vertical_distance = (mls_grid->getScaleX() + mls_grid->getScaleY()) * 0.5;
    if(vertical_distance <= 0.0)
	vertical_distance = 0.1;
    
    // create pointcloud from mls
    for(size_t x=0;x<mls_grid->getCellSizeX();x++)
    {
	for(size_t y=0;y<mls_grid->getCellSizeY();y++)
	{
	    for( MLSGrid::iterator cit = mls_grid->beginCell(x,y); cit != mls_grid->endCell(); cit++ )
	    {
		MLSGrid::SurfacePatch p( *cit );
		Eigen::Vector3d cellPosWorld = mls_grid->fromGrid(x, y, mls_grid->getEnvironment()->getRootNode());
		Eigen::Vector3d point(cellPosWorld);
		
		if(p.isHorizontal())
		{
		    point[2] = cellPosWorld.z() + p.mean;
		    pointcloud->vertices.push_back(point);
		    
		}
		else if(p.isVertical())
		{
		    float min_z = (float)p.getMinZ(0);
		    float max_z = (float)p.getMaxZ(0);
		    for(float z = min_z; z <= max_z; z += vertical_distance)
		    {
			point[2] = cellPosWorld.z() + z;
			pointcloud->vertices.push_back(point);
		    }
		}
	    }
	}
    }
    
    pointcloud->itemModified();
    return true;
}
