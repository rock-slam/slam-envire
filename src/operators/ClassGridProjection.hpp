#ifndef envire_CLASS_GRID_PROJECTION_HH
#define envire_CLASS_GRID_PROJECTION_HH

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>

namespace envire {
    /** Classification of terrain into symbolic traversability classes, based on
     * different modalities.
     *
     * For now, the modalities that are used are:
     *
     * <ul>
     * <li>terrain's maximum allowed torque
     * <li>local slope
     * <li>maximum step size
     * </ul>
     *
     * It outputs a map in which each cell has an integer value, this integer
     * value being the traversability class for the cell
     *
     * If one of the modality is missing, it is simply ignored
     */
    template<typename Input, typename Output>
    class ClassGridProjection : public envire::Operator {
    public:
        std::map<Input, Output> class_map;

        ClassGridProjection()
            : envire::Operator(1, 1) {}

        bool updateAll()
        {
            envire::Grid<Input> const* input = getInput< envire::Grid<Input> const* >();
            envire::Grid<Output>* output  = getOutput< envire::Grid<Output>* >();

            if (!input->isAlignedWith(*output))
                throw std::runtime_error("trying to apply ClassGridProjection on " + input->getUniqueId() + " -> " + output->getUniqueId() + ", which are not aligned with each other");

            boost::multi_array<Input, 2> const& input_data = input->getGridData();
            boost::multi_array<Output, 2>& output_data = output->getGridData();

            int xSize = input->getCellSizeX(), ySize = input->getCellSizeY();
            for (int y = 0; y < ySize; ++y)
            {
                Input const* input_line = &input_data[y][0];
                Output* output_line = &output_data[y][0];
                for (int x = 0; x < xSize; ++x)
                {
                    typename std::map<Input, Output>::const_iterator it = class_map.find(input_line[x]);
                    if (it == class_map.end())
                        throw std::runtime_error("found class " + boost::lexical_cast<std::string>(input_line[x]) + ", for with there is no projection information");

                    output_line[x] = it->second;
                }
            }

            return true;
        }
    };
}

#endif

