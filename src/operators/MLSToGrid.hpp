#ifndef __ENVIRE__MLS_SLOPE_HPP__
#define __ENVIRE__MLS_SLOPE_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>

namespace envire
{
    /** A very stupid and limited MLS-to-grid convertion operator
     *
     * It acts on an MLSGrid and updates a Grid<double> with the highest point
     * in the MLS at this cell
     */
    class MLSToGrid : public Operator
    {
	ENVIRONMENT_ITEM( MLSToGrid )

        std::string mOutLayerName;

    public:
        MLSToGrid();
        ~MLSToGrid();

	void serialize( Serialization &so );
	void unserialize( Serialization &so );

        void setOutput(Grid<double>* grid, std::string const& name);
	bool updateAll();
    };
}

#endif
