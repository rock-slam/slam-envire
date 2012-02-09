#ifndef __ENVIRE_GRIDTOMLS_HPP__
#define __ENVIRE_GRIDTOMLS_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>
#include <envire/maps/MLSGrid.hpp>

#include <Eigen/Core>

namespace envire {
    /** Operator that converts a Grid<float> or Grid<double> to a MLSGrid
     */
    class GridFloatToMLS : public Operator
    {
	ENVIRONMENT_ITEM( GridFloatToMLS )

    private:
        using Operator::addInput;
        using Operator::addOutput;

    public:
	GridFloatToMLS();
	void serialize(Serialization& so);
        void unserialize(Serialization& so);

        /** @overload
         *
         * Calls either Grid<float> or Grid<double> based on the actual type of
         * \c grid. Will throw std::runtime_error if the grid is of a different
         * type
         */
	void setInput( GridBase* grid, std::string const& band = "" ); 
        /** Sets the input of this operator as a Grid<float>
         *
         * @arg grid the input grid
         * @arg band the band that should be used in \c grid. Leave empty if the
         *           grid has a single band
         */
	void setInput( Grid<float>* grid, std::string const& band = "" ); 
        /** Sets the input of this operator as a Grid<double>
         *
         * @arg grid the input grid
         * @arg band the band that should be used in \c grid. Leave empty if the
         *           grid has a single band
         */
	void setInput( Grid<double>* grid, std::string const& band = "" ); 
        /** Sets the MLS output of this operator */
	void setOutput( MLSGrid* grid ); 

        /** Applies the operator on the configured inputs and outputs */
	bool updateAll();

    private:
        std::string band;
    };
}
#endif

