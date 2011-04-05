#include "Traversability.hpp"
#include <envire/maps/MultiLevelSurfaceGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/multi_array.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( Traversability )

bool Traversability::updateAll() 
{
    // this implementation can handle only one input at the moment
    if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
        throw std::runtime_error("Traversability needs to have exactly 1 input and 1 output for now.");
    
    TraversabilityGrid* travGrid = dynamic_cast<envire::TraversabilityGrid*>(*env->getOutputs(this).begin());
    Layer* layer = *env->getInputs(this).begin();
    if( dynamic_cast<envire::MultiLevelSurfaceGrid*>(layer) )
    {
	// input is mls grid
	MultiLevelSurfaceGrid* mls = dynamic_cast<envire::MultiLevelSurfaceGrid*>(layer); 
	assert( mls->getWidth() == travGrid->getWidth() && mls->getHeight() == travGrid->getHeight() );
	assert( mls->getScaleX() == travGrid->getScaleX() && mls->getScaleY() == travGrid->getScaleY() );
	
	// init traversibility grid
	boost::multi_array<uint8_t,2>& trav(travGrid->getGridData(TraversabilityGrid::TRAVERSABILITY));
	std::fill(trav.data(), trav.data() + trav.num_elements(), 255);

	size_t width = mls->getWidth(); 
	size_t height = mls->getHeight(); 

	double scalex = mls->getScaleX();
	double scaley = mls->getScaleY();

	for(size_t x=1;x<(width-1);x++)
	{
	    for(size_t y=1;y<(height-1);y++)
	    {
		double max_grad = -std::numeric_limits<double>::infinity();

		MultiLevelSurfaceGrid::iterator maxc = 
		    std::max_element( mls->beginCell(x,y), mls->endCell() );
		
		if( maxc != mls->endCell() )
		{
		    for(int i=0;i<3;i++)
		    {
			// get the 4 neighbours
			int m = (i<2)*(i*2-1);
			int n = (i>=2)*((i-2)*2-1);
			double scale = (i<2)?scalex:scaley;

			MultiLevelSurfaceGrid::iterator max = 
			    std::max_element( mls->beginCell(x+m,y+n), mls->endCell() );

			if( max != mls->endCell() )
			{
			    double gradient = (max->mean - maxc->mean) / scale;
			    max_grad = std::max( max_grad, gradient );
			}
		    }
		}
		double angle = atan( max_grad );

		// traversibility is only based on angle for now
		// should include other stuff later
		trav[x][y] = ((uint8_t)(angle/(M_PI/2.0)*6.0));
	    }
	}
    }

    return true;
}
