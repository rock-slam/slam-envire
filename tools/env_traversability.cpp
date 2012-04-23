#include "envire/Core.hpp"
#include "envire/maps/Grid.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/operators/SimpleTraversability.hpp"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 2 || argc > 8)
    {
	std::cout << "usage: env_traversability input mass force_threshold [class_count [min_width [ground_clearance]]]" << std::endl;
        std::cout << "  generates a classified map of traversabilities based on the given map (a single map must contain all the necessary bands)" << std::endl;
        std::cout << "  the generated map will have the same width, height and cell size than the source map" << std::endl;
        std::cout << std::endl;
        std::cout << "  If the parameters are given, a new operator and map are created" << std::endl;
        std::cout << "  use env_update to re-run an existing operator in an environment" << std::endl;
	exit(0);
    }
    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));

    std::string slope_band = "mean_slope";
    std::string step_band  = "corrected_max_step";
    std::string force_band = "max_force";

    // Check if there is already an output map and operator that match what is
    // asked
    boost::intrusive_ptr< Grid<float> > input = env->getItem< Grid<float> >();

    env->updateOperators();

    boost::intrusive_ptr< SimpleTraversability::OutputLayer > output = new SimpleTraversability::OutputLayer(
            input->getWidth(), input->getHeight(),
            input->getScaleX(), input->getScaleY());
    env->attachItem(output.get());
    output->setFrameNode(input->getFrameNode());

    double weight_force = boost::lexical_cast<double>(argv[2]) * 9.81;
    double force_threshold = boost::lexical_cast<double>(argv[3]);
    int class_count = 10;
    if (argc > 4)
        class_count = boost::lexical_cast<int>(argv[5]);

    double min_width = 0.5;
    if (argc > 5)
        min_width = boost::lexical_cast<double>(argv[6]);

    double ground_clearance = 0;
    if (argc > 6)
        ground_clearance = boost::lexical_cast<double>(argv[7]);

    // Create the convertion operator and run it
    envire::SimpleTraversability *op = new SimpleTraversability(
            weight_force, force_threshold, class_count, min_width, ground_clearance);
    env->attachItem( op );
    if (input->hasBand("mean_slope"))
        op->setSlope(input.get(), "mean_slope");
    if (input->hasBand("corrected_max_step"))
        op->setMaxStep(input.get(), "corrected_max_step");
    if (input->hasBand("max_force"))
        op->setMaxForce(input.get(), "max_force");
    op->setOutput(output.get(), "traversability_class");
    op->updateAll();

    env->serialize(argv[1]);
} 

