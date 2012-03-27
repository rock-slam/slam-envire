#include "MergeMLS.hpp"
#include <envire/maps/MLSMap.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( MergeMLS )

bool MergeMLS::updateAll()
{
    MLSGrid* output = static_cast<envire::MLSGrid*>(*env->getOutputs(this).begin());
    assert(output);

    // get the maps from the input
    // inputs can either be grids directly, or maps which
    // have grids as children
    std::vector<MLSGrid*> grids;
    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	MLSGrid* grid = dynamic_cast<envire::MLSGrid*>(*it);
	if( grid )
	    grids.push_back( grid );
	else
	{
	    MLSMap* map = dynamic_cast<envire::MLSMap*>(*it);
	    if( map )
	    {
		std::list<Layer*> children = env->getChildren( map );
		for( std::list<Layer*>::iterator it2 = children.begin(); it2 != children.end(); it2++ )
		{
		    MLSGrid* grid = dynamic_cast<envire::MLSGrid*>(*it2);
		    assert( grid );
		    grids.push_back( grid );
		}
	    }
	    else
		throw std::runtime_error("Not a valid input to MLSMerge operator");
	}
    }

    // now for each cell in the input grid, 
    // merge into the output grid
    for( std::vector<MLSGrid*>::iterator it = grids.begin(); it != grids.end(); it++ )
    {
	MLSGrid* input = *it;

	FrameNode::TransformType C_m2g = env->relativeTransform( input->getFrameNode(), output->getFrameNode() );
	// we support only translations for now
	assert( C_m2g.linear().isIdentity() );
	Eigen::Vector3d trans = C_m2g.translation();

	for(size_t m=0;m<input->getWidth();m++)
	{
	    for(size_t n=0;n<input->getHeight();n++)
	    {
		for( MLSGrid::const_iterator cit = input->beginCell(m,n); cit != input->endCell(); cit++ )
		{
		    MLSGrid::SurfacePatch p( *cit );
		    p.mean += trans.z();
		    output->updateCell( m, n, p );
		}
	    }
	}
    }

    return true;
}
