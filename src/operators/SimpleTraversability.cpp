#include "SimpleTraversability.hpp"
#include <base-logging/Logging.hpp>
#include <sstream>

using namespace envire;
using envire::Grid;

ENVIRONMENT_ITEM_DEF( SimpleTraversability );
/* For backward compatibility reasons */
static envire::SerializationPlugin< SimpleTraversability >  nav_graph_search_TraversabilityClassifier("nav_graph_search::TraversabilityClassifier");
/* For backward compatibility reasons */
static envire::SerializationPlugin< SimpleTraversability >  envire_MLSSimpleTraversability("envire::MLSSimpleTraversability");

SimpleTraversability::SimpleTraversability()
{
}

SimpleTraversability::SimpleTraversability(
        SimpleTraversabilityConfig const& conf)
    : Operator(0, 1)
    , conf(conf)
{
}

SimpleTraversability::SimpleTraversability(
        double maximum_slope,
        int class_count,
        double min_width,
        double ground_clearance)
    : Operator(0, 1)
{
    conf.maximum_slope     = maximum_slope;
    conf.class_count      = class_count;
    conf.min_width        = min_width;
    conf.ground_clearance = ground_clearance;
}

void SimpleTraversability::serialize(envire::Serialization& so)
{
    Operator::serialize(so);

    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        if (input_layers_id[i] != "" && !input_bands[i].empty())
        {
            so.write("input" + boost::lexical_cast<std::string>(i), input_layers_id[i]);
            so.write("input" + boost::lexical_cast<std::string>(i) + "_band", input_bands[i]);
        }
    }

    so.write("maximum_slope", conf.maximum_slope);
    so.write("class_count", conf.class_count);
    so.write("ground_clearance", conf.ground_clearance);
    so.write("min_width", conf.min_width);
    so.write("output_band", output_band);
}

void SimpleTraversability::unserialize(envire::Serialization& so)
{
    Operator::unserialize(so);
    
    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        std::string input_key = "input" + boost::lexical_cast<std::string>(i);
        if (so.hasKey(input_key))
        {
            input_layers_id[i] = so.read<std::string>(input_key);
            input_bands[i] = so.read<std::string>(input_key + "_band");
        }
    }

    so.read<double>("maximum_slope", conf.maximum_slope);
    so.read<int>("class_count", conf.class_count);
    so.read<double>("ground_clearance", conf.ground_clearance);
    so.read<double>("min_width", conf.min_width);
    so.read<std::string>("output_band", output_band);
}

envire::Grid<float>* SimpleTraversability::getInputLayer(INPUT_DATA index) const
{
    if (input_layers_id[index] == "")
        return 0;
    return getEnvironment()->getItem< Grid<float> >(input_layers_id[index]).get();
}
std::string SimpleTraversability::getInputBand(INPUT_DATA index) const
{ return input_bands[index]; }

envire::Grid<float>* SimpleTraversability::getSlopeLayer() const { return getInputLayer(SLOPE); }
std::string SimpleTraversability::getSlopeBand() const { return getInputBand(SLOPE); }
void SimpleTraversability::setSlope(Grid<float>* grid, std::string const& band_name)
{
    addInput(grid);
    input_layers_id[SLOPE] = grid->getUniqueId();
    input_bands[SLOPE] = band_name;

}

envire::Grid<float>* SimpleTraversability::getMaxStepLayer() const { return getInputLayer(MAX_STEP); }
std::string SimpleTraversability::getMaxStepBand() const { return getInputBand(MAX_STEP); }
void SimpleTraversability::setMaxStep(Grid<float>* grid, std::string const& band_name)
{
    addInput(grid);
    input_layers_id[MAX_STEP] = grid->getUniqueId();
    input_bands[MAX_STEP] = band_name;
}

void SimpleTraversability::setOutput(OutputLayer* grid, std::string const& band_name)
{
    removeOutputs();
    addOutput(grid);
    output_band = band_name;
}

bool SimpleTraversability::updateAll()
{
    OutputLayer* output_layer = getOutput< OutputLayer* >();
    if (!output_layer)
        throw std::runtime_error("SimpleTraversability: no output band set");

    OutputLayer::ArrayType& result = output_band.empty() ?
        output_layer->getGridData() :
        output_layer->getGridData(output_band);

    if (output_band.empty())
        output_layer->setNoData(CLASS_UNKNOWN);
    else
        output_layer->setNoData(output_band, CLASS_UNKNOWN);

    static float const DEFAULT_UNKNOWN_INPUT = -std::numeric_limits<float>::infinity();
    Grid<float> const* input_layers[INPUT_COUNT] = { 0, 0};
    float input_unknown[INPUT_COUNT];
    //init probability with zero
    TraversabilityGrid::ArrayType &probabilityArray(output_layer->getGridData(TraversabilityGrid::PROBABILITY));
    std::fill(probabilityArray.data(), probabilityArray.data() + probabilityArray.num_elements(), 0);  

    boost::multi_array<float, 2> const* inputs[INPUT_COUNT] = { 0, 0 };
    bool has_data = false;
    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        if (input_layers_id[i] != "" && !input_bands[i].empty())
        {
            input_layers[i] = getEnvironment()->getItem< Grid<float> >(input_layers_id[i]).get();
            has_data = true;
            inputs[i] = &(input_layers[i]->getGridData(input_bands[i]));

            std::pair<float, bool> no_data = input_layers[i]->getNoData(input_bands[i]);
            if (no_data.second)
            {
//                 std::cout << "band " << i << " no_data=" << no_data.first << std::endl;
                input_unknown[i] = no_data.first;
            }
            else
                input_unknown[i] = DEFAULT_UNKNOWN_INPUT;
        }
    }
    if (!has_data)
        throw std::runtime_error("SimpleTraversability: no input layer configured");

    //if (inputs[MAX_STEP] && conf.ground_clearance == 0)
    //    throw std::runtime_error("a max_step band is available, but the ground clearance is set to zero");

    int width = output_layer->getWidth(), height = output_layer->getHeight();
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // Read the values for this cell. Set to CLASS_UNKNOWN and ignore the cell
            // if one of the available input bands has no information
            double values[INPUT_COUNT];
            bool has_value[INPUT_COUNT];
            for (int band_idx = 0; band_idx < INPUT_COUNT; ++band_idx)
            {
                has_value[band_idx] = false;
                if (!inputs[band_idx]) 
                    continue;

                double value = (*inputs[band_idx])[y][x];
                if (value != input_unknown[band_idx])
                {
                    values[band_idx] = value;
                    has_value[band_idx] = true;
                }
            }

            bool has_slope = has_value[SLOPE],
                 has_max_step    = has_value[MAX_STEP];

            // First, max_step is an ON/OFF threshold on the ground clearance
            // parameter
            if (has_max_step && conf.ground_clearance && (values[MAX_STEP] > conf.ground_clearance))
            {
                result[y][x] = CLASS_OBSTACLE;
                probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();
                continue;
            }
            
            if (inputs[SLOPE] && !has_slope)
            {
                result[y][x] = CLASS_UNKNOWN;
                continue;
            }

            if(has_slope && conf.maximum_slope)
            {
                const double meanSlope(fabs(values[SLOPE]));
                if(meanSlope > conf.maximum_slope)
                {
                    result[y][x] = CLASS_OBSTACLE;
                    probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();

                    continue;
                }
                else
                {
                    //scale current slope to cost classes
                    int klass = conf.class_count - rint(meanSlope / conf.maximum_slope * conf.class_count);
                    result[y][x] = CUSTOM_CLASSES + klass;
                    probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();
                }
            }
        }
    }
    
    // perform some post processing if required
    if( conf.min_width > 0 ) 
    {
        closeNarrowPassages(*output_layer, output_band, conf.min_width);
    }

    if( conf.obstacle_clearance > 0 ) 
    {
        growObstacles(*output_layer, output_band, conf.obstacle_clearance);
    }
	  
	// Registers klasses in traversability map.
    output_layer->setTraversabilityClass(CLASS_OBSTACLE, TraversabilityClass(0));
    double driveability = 0.0;
    for(int i = 0; i <= conf.class_count; i++)
    {
        driveability = (1.0 / conf.class_count) + 
                (1.0 - (1.0 / conf.class_count)) / conf.class_count * (i);
        output_layer->setTraversabilityClass(CUSTOM_CLASSES + i, TraversabilityClass(driveability));
    }
    
    // Calculates the mean traversability class of the current map ignoring unknown areas.
    uint8_t class_value = 0;
    double sum_classes = 0.0;
    double counter = 0.0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            class_value = result[y][x];
            // Sum up if not unknown.
            if(class_value != CLASS_UNKNOWN) {
                sum_classes += class_value;
                counter++;
            }
        }
    }

    // Traversability class 7 contains the mean driveability (0.55).
    double mean_driveability = output_layer->getTraversabilityClass(7).getDrivability();
    uint8_t mean_class_value = counter == 0 ? 0 : sum_classes / counter + 0.5;
    
    // If the map is completely unkown or full with obstacles, 
    // set the driveability of unknown areas to the mean driveability.    
    if(mean_class_value <= 1) {
        output_layer->setTraversabilityClass(CLASS_UNKNOWN, TraversabilityClass(mean_driveability)); 
    } else {
        TraversabilityClass mean_trav_class = output_layer->getTraversabilityClass(mean_class_value);
        output_layer->setTraversabilityClass(CLASS_UNKNOWN, mean_trav_class);
    }
    
    std::stringstream ss;
    ss << "Traversability classes:" << std::endl;
    for(int i = 0; i <= CUSTOM_CLASSES + conf.class_count; i++)
    {
        ss << "Driveability of traversability class " << i << ": " << 
                output_layer->getTraversabilityClass(i).getDrivability() << std::endl;
    }
    LOG_INFO(ss.str().c_str());

    return true;
}

struct RadialLUT
{
    int centerx, centery;
    unsigned int width, height;
    boost::multi_array<std::pair<int, int>, 2>  parents;
    boost::multi_array<bool, 2> in_distance;

    void precompute(double distance, double scalex, double scaley)
    {
        double const radius2 = distance * distance;

        width  = 2* ceil(distance / scalex) + 1;
        height = 2* ceil(distance / scaley) + 1;
        in_distance.resize(boost::extents[height][width]);
        std::fill(in_distance.data(), in_distance.data() + in_distance.num_elements(), false);
        parents.resize(boost::extents[height][width]);
        std::fill(parents.data(), parents.data() + parents.num_elements(), std::make_pair(-1, -1));

        centerx = width  / 2;
        centery = height / 2;
        parents[centery][centerx] = std::make_pair(-1, -1);

        for (unsigned int y = 0; y < height; ++y)
        {
            for (unsigned int x = 0; x < width; ++x)
            {
                int dx = (centerx - x);
                int dy = (centery - y);
                if (dx == 0 && dy == 0) continue;

                double d2 = dx * dx * scalex * scalex + dy * dy * scaley * scaley;
                in_distance[y][x] = (d2 < radius2);
                if (abs(dx) > abs(dy))
                {
                    int parentx = x + dx / abs(dx);
                    int parenty = y + rint(static_cast<double>(dy) / abs(dx));
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
                else
                {
                    int parentx = x + rint(static_cast<double>(dx) / abs(dy));
                    int parenty = y + dy / abs(dy);
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
            }
        }
    }

    void markAllRadius(boost::multi_array<uint8_t, 2>& result, TraversabilityGrid::ArrayType &probabilityArray, int result_width, int result_height, int centerx, int centery, int value)
    {
        int base_x = centerx - this->centerx;
        int base_y = centery - this->centery;
        for (unsigned int y = 0; y < height; ++y)
        {
            int map_y = base_y + y;
            if (map_y < 0 || map_y >= result_height)
                continue;

            for (unsigned int x = 0; x < width; ++x)
            {
                int map_x = base_x + x;
                if (map_x < 0 || map_x >= result_width)
                    continue;
                if (in_distance[y][x] && result[map_y][map_x] == value)
                {
//                     LOG_DEBUG("  found cell with value %i (expected %i) at %i %i, marking radius", result[map_y][map_x], value, map_x, map_y);
                    markSingleRadius(result, probabilityArray, centerx, centery, x, y, value, 255);
                }
            }
        }
    }

    void markSingleRadius(boost::multi_array<uint8_t, 2>& result, TraversabilityGrid::ArrayType &probabilityArray, int centerx, int centery, int x, int y, int expected_value, int mark_value)
    {
        boost::tie(x, y) = parents[y][x];
        while (x != -1 && y != -1)
        {
            int map_x = centerx + x - this->centerx;
            int map_y = centery + y - this->centery;
            uint8_t& current = result[map_y][map_x];
            if (current != expected_value)
	    {
		current = mark_value;
// 		LOG_DEBUG("  marking %i %i", map_x, map_y);
                probabilityArray[map_y][map_x] = std::numeric_limits<uint8_t>::max();
	    }
            boost::tie(x, y) = parents[y][x];
        }
    }
};

void SimpleTraversability::growObstacles(OutputLayer& map, std::string const& band_name, double width)
{
    const double width_square = pow(width,2);
    const int 
	wx = width / map.getScaleX(), 
	wy = width / map.getScaleY();
    const double 
	sx = map.getScaleX(),
	sy = map.getScaleY();


    OutputLayer::ArrayType& orig_data = band_name.empty() ?
        map.getGridData() :
        map.getGridData(output_band);

    OutputLayer::ArrayType data( orig_data );
    TraversabilityGrid::ArrayType &probabilityArray(map.getGridData(TraversabilityGrid::PROBABILITY));

    for (unsigned int y = 0; y < map.getHeight(); ++y)
    {
        for (unsigned int x = 0; x < map.getWidth(); ++x)
        {
            int value = orig_data[y][x];
            if (value == CLASS_OBSTACLE)
            {
		// make everything with radius width around the obstacle also
		// an obstacle
		for( int oy = -wy; oy <= wy; ++oy )
		{
		    for( int ox = -wx; ox <= wx; ++ox )
		    {
			const int tx = x+ox;
			const int ty = y+oy;
			if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
				&& tx >= 0 && tx < (int)map.getWidth()
				&& ty >= 0 && ty < (int)map.getHeight() )
                        {
			    data[ty][tx] = CLASS_OBSTACLE;
                            probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();
                        }
		    }
		}
	    }
	}
    }

    std::swap( data, orig_data );
}

void SimpleTraversability::closeNarrowPassages(SimpleTraversability::OutputLayer& map, std::string const& band_name, double min_width)
{
    RadialLUT lut;
    lut.precompute(min_width, map.getScaleX(), map.getScaleY());
    std::stringstream oss;
    oss << std::endl;
    for (unsigned int y = 0; y < lut.height; ++y)
    {
        for (unsigned int x = 0; x < lut.width; ++x) {
            oss << "(" << lut.parents[y][x].first << " " << lut.parents[y][x].second << ") ";
        }
        oss << std::endl;
    }
    oss << std::endl;
    for (unsigned int y = 0; y < lut.height; ++y)
    {
        for (unsigned int x = 0; x < lut.width; ++x) {
            oss << "(" << lut.in_distance[y][x] << " ";
        }
        oss << std::endl;
    }
    LOG_DEBUG(oss.str().c_str());

    TraversabilityGrid::ArrayType &probabilityArray(map.getGridData(TraversabilityGrid::PROBABILITY));

    OutputLayer::ArrayType& data = band_name.empty() ?
        map.getGridData() :
        map.getGridData(output_band);
    for (unsigned int y = 0; y < map.getHeight(); ++y)
    {
        for (unsigned int x = 0; x < map.getWidth(); ++x)
        {
            int value = data[y][x];
            if (value == CLASS_OBSTACLE)
            {
//                 LOG_DEBUG("inspecting around obstacle cell %i %i", x, y);
                lut.markAllRadius(data, probabilityArray, map.getWidth(), map.getHeight(), x, y, CLASS_OBSTACLE);
            }
        }
    }

    for (size_t y = 0; y < map.getHeight(); ++y)
    {
        for (size_t x = 0; x < map.getWidth(); ++x)
        {
            if (data[y][x] == 255)
            {
                data[y][x] = CLASS_OBSTACLE;
                probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();
            }
        }
    }
}

