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

    if( !reverse )
    {
	// now for each cell in the input grid, 
	// merge into the output grid
	for( std::vector<MLSGrid*>::iterator it = grids.begin(); it != grids.end(); it++ )
	{
	    MLSGrid* input = *it;

	    FrameNode::TransformType C_m2g = env->relativeTransform( input->getFrameNode(), output->getFrameNode() );

	    for(size_t m=0;m<input->getWidth();m++)
	    {
		for(size_t n=0;n<input->getHeight();n++)
		{
		    for( MLSGrid::iterator cit = input->beginCell(m,n); cit != input->endCell(); cit++ )
		    {
			MLSGrid::SurfacePatch p( *cit );

			// get 3d position of gridcell
			Eigen::Vector3d pos;
			pos << input->fromGrid( GridBase::Position( m, n ) ), p.mean;

			// transform into target frame
			Eigen::Vector3d target_pos = C_m2g * pos;

			// see if available in target grid
			MLSGrid::Position t_pos;
			if( output->toGrid( target_pos.head<2>(), t_pos ) )
			{
			    p.mean = target_pos.z();
			    output->updateCell( t_pos.x, t_pos.y, p );
			}
		    }
		}
	    }
	}
    }
    else
    {
	// the reverse case is used to prevent aliasing, and can
	// be used if the target output resolution is smaller than the input.
	// It's a little slower however
	
	// precalculate the required transforms
	std::vector<base::Transform3d> transforms;
	for( std::vector<MLSGrid*>::iterator it = grids.begin(); it != grids.end(); it++ )
	{
	    MLSGrid* input = *it;
	    FrameNode::TransformType C_g2m = 
		env->relativeTransform( output->getFrameNode(), input->getFrameNode() );
	    transforms.push_back( C_g2m );
	}

	for(size_t m=0;m<output->getWidth();m++)
	{
	    for(size_t n=0;n<output->getHeight();n++)
	    {
		// get 3d position of output gridcell
		Eigen::Vector3d pos;
		pos << output->fromGrid( GridBase::Position( m, n ) ), 0;

		// go through the input grids
		// and have a look if we get a mapping
		for( size_t t=0; t<grids.size(); t++ )
		{
		    MLSGrid* input = grids[t];

		    // transform into target frame
		    Eigen::Vector3d src_pos = transforms[t] * pos;

		    // see if available in target grid
		    MLSGrid::Position s_pos;
		    if( input->toGrid( src_pos.head<2>(), s_pos ) )
		    {
			for( MLSGrid::iterator cit = input->beginCell(s_pos.x, s_pos.y); cit != input->endCell(); cit++ )
			{
			    MLSGrid::SurfacePatch p( *cit );
			    p.mean += src_pos.z();

			    output->updateCell( m, n, p );
			}
		    }
		}
	    }
	}
    }

    return true;
}
