#include <envire/operators/GridFloatToMLS.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( GridFloatToMLS )

GridFloatToMLS::GridFloatToMLS()
    : Operator(1, 1), 
    uncertainty( 0.1 )
{
}

void GridFloatToMLS::setGridUncertainty(float uncertainty)
{
    this->uncertainty = uncertainty;
}

void GridFloatToMLS::unserialize(Serialization& so)
{
    Operator::unserialize(so);
    so.read<std::string>("band_name", band);
}

void GridFloatToMLS::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.write("band_name", band);
}

void GridFloatToMLS::setInput( GridBase* grid, std::string const& band) 
{
    if (!dynamic_cast< Grid<double>* >(grid) && !dynamic_cast< Grid<float>* >(grid))
        throw std::runtime_error("GridFloatToMLS expects either a Grid<float> or a Grid<double>");
    this->band = band;
    Operator::addInput(grid);
}

void GridFloatToMLS::setInput( Grid<float>* grid, std::string const& band) 
{
    setInput(static_cast<GridBase*>(grid), band);
}

void GridFloatToMLS::setInput( Grid<double>* grid, std::string const& band) 
{
    setInput(static_cast<GridBase*>(grid), band);
}

void GridFloatToMLS::setOutput( MLSGrid* mls )
{
    if( env->getOutputs(this).size() > 0 )
        throw std::runtime_error("GridFloatToMLS can only have one output.");
    Operator::addOutput(mls);
}

template<typename T>
static void convert(Grid<T>* grid, std::string const& band_name, MLSGrid* mls, float uncertainty )
{
    boost::multi_array<T, 2>* grid_data;
    if (band_name.empty())
        grid_data = &grid->getGridData();
    else
        grid_data = &grid->getGridData(band_name);

    // try to find rgb bands in input
    boost::multi_array<T, 2> *r = NULL, *g = NULL, *b = NULL;
    if( grid->hasBand( "r" ) )
	r = &grid->getGridData( "r" );
    if( grid->hasBand( "g" ) )
	g = &grid->getGridData( "g" );
    if( grid->hasBand( "b" ) )
	b = &grid->getGridData( "b" );
    bool hasColor = false;
    if( r && g && b )
    {
	hasColor = true;
	mls->setHasCellColor( true );
    }

    // we can now iterate through the source, or the target grid
    // This should be depending on the cell resolution

    if( mls->getScaleX() <= grid->getScaleX() )
    {
	Transform mls2grid = grid->getEnvironment()->relativeTransform( grid, mls );
    
	for (size_t yi = 0; yi < mls->getCellSizeY(); ++yi)
	{
	    double y = mls->getScaleY() * yi + mls->getOffsetY();
	    for (size_t xi = 0; xi < mls->getCellSizeX(); ++xi)
	    {
		double x = mls->getScaleX() * xi + mls->getOffsetX();
		Eigen::Vector3d src_p = mls2grid * Eigen::Vector3d(x, y, 0);
		size_t src_xi, src_yi;
		if (!grid->toGrid(src_p.x(), src_p.y(), src_xi, src_yi))
		    continue;

		T value = (*grid_data)[src_yi][src_xi];
		envire::SurfacePatch updatePatch( value, uncertainty );
		if( hasColor )
		    updatePatch.setColor( Eigen::Vector3d( (*r)[src_yi][src_xi], (*g)[src_yi][src_xi], (*b)[src_yi][src_xi] ) );
		mls->updateCell(xi, yi, updatePatch);
	    }
	}
    }
    else  
    {
	Transform grid2mls = grid->getEnvironment()->relativeTransform( mls, grid );
    
	for (size_t yi = 0; yi < grid->getCellSizeY(); ++yi)
	{
	    double y = grid->getScaleY() * yi + grid->getOffsetY();
	    for (size_t xi = 0; xi < grid->getCellSizeX(); ++xi)
	    {
		double x = grid->getScaleX() * xi + grid->getOffsetX();
		Eigen::Vector3d mls_p = grid2mls * Eigen::Vector3d(x, y, 0);

		T value = (*grid_data)[yi][xi];
		envire::SurfacePatch updatePatch( value, uncertainty );
		if( hasColor )
		    updatePatch.setColor( Eigen::Vector3d( (*r)[yi][xi], (*g)[yi][xi], (*b)[yi][xi] ) );
		mls->update( mls_p.head<2>(), updatePatch );
	    }
	}
    }
}

bool GridFloatToMLS::updateAll()
{
    MLSGrid* mls = static_cast<MLSGrid*>(*env->getOutputs(this).begin());

    Grid<float>* grid = dynamic_cast<Grid<float>*>(*env->getInputs(this).begin());
    if (grid)
        convert(grid, band, mls, uncertainty);
    else
    {
        Grid<double>* grid = dynamic_cast<Grid<double>*>(*env->getInputs(this).begin());
        if (grid)
            convert(grid, band, mls, uncertainty);
        else
            throw std::logic_error("could not find an input of either type Grid<double> or Grid<float>");
    }

    env->itemModified(mls);
    return true;
}
