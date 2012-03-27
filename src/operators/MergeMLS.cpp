#include "MergeMLS.hpp"

using namespace envire;

ENVIRONMENT_ITEM_DEF( MergeMLS )

bool MergeMLS::updateAll()
{
    MLSGrid* output = static_cast<envire::MLSGrid*>(*env->getOutputs(this).begin());
    assert(output);

    std::list<Layer*> inputs = env->getInputs(this);
    for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
    {
	MLSGrid* input = dynamic_cast<envire::MLSGrid*>(*it);
	assert(input);

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
