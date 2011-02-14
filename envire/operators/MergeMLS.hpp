#ifndef __ENVIRE_OPERATORS_MERGEMLS__
#define __ENVIRE_OPERATORS_MERGEMLS__

#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>

namespace envire 
{

class MergeMLS : public Operator
{
    ENVIRONMENT_ITEM( MergeMLS )

public:
    MergeMLS() {};
    MergeMLS( Serialization& so ) : Operator( so ) {}

    bool updateAll()
    {
	MultiLevelSurfaceGrid* output = static_cast<envire::MultiLevelSurfaceGrid*>(*env->getOutputs(this).begin());
	assert(output);

	std::list<Layer*> inputs = env->getInputs(this);
	for( std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
	{
	    MultiLevelSurfaceGrid* input = dynamic_cast<envire::MultiLevelSurfaceGrid*>(*it);
	    assert(input);

	    FrameNode::TransformType C_m2g = env->relativeTransform( input->getFrameNode(), output->getFrameNode() );
	    // we support only translations for now
	    assert( C_m2g.linear().isIdentity() );
	    Eigen::Vector3d trans = C_m2g.translation();

	    for(size_t m=0;m<input->getWidth();m++)
	    {
		for(size_t n=0;n<input->getHeight();n++)
		{
		    for( MultiLevelSurfaceGrid::const_iterator cit = input->beginCell_const(m,n); cit != input->endCell_const(); cit++ )
		    {
			MultiLevelSurfaceGrid::SurfacePatch p( *cit );
			p.mean += trans.z();
			output->updateCell( m, n, p );
		    }
		}
	    }
	}

	return true;
    };
};
}
#endif
