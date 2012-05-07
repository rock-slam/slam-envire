#ifndef envire_MLS_SIMPLE_TRAVERSABILITY_CLASSIFIER_HH
#define envire_MLS_SIMPLE_TRAVERSABILITY_CLASSIFIER_HH

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>

namespace envire {
    /** @brief Configuration parameters for the SimpleTraversability operator
     *
     * See SimpleTraversability for more information on the use of these
     * parameters.
     */
    struct SimpleTraversabilityConfig
    {
        /** Force (in N) due to system's mass */
        double weight_force;

        /** Force (in N) that constitutes the highest class */
        double force_threshold;

        /** Number of classes in the output map */
        int class_count;

        /** Traversable passages that are narrower (in m) than this will be
         * closed */
        double min_width;

        /** Required ground clearance, in meters. A cell is classified as
         * obstacle if the maximum step around that cell is higher than the
         * ground clearance
         */
       double ground_clearance;

       SimpleTraversabilityConfig()
           : weight_force(0)
           , force_threshold(0)
           , class_count(0)
           , min_width(0)
           , ground_clearance(0) {}
    };

    /** @brief Classification of terrain into symbolic traversability classes, based on
     * achievable traction force and geometric constraints
     *
     * For now, the modalities that are used are:
     *
     * <ul>
     * <li>terrain's maximum allowed torque
     * <li>local slope
     * <li>maximum step size
     * </ul>
     *
     * The (very simple) model computes the maximum local traction force and
     * maps the result from [0, force_threshold] to [0, 1]. The resulting value
     * is then quantified in @a class_count intervals.
     *
     * It outputs a map in which each cell has an integer value, this integer
     * value being the traversability class for the cell. Since CLASS_UNKNOWN
     * (=0) and CLASS_OBSTACLE (=1) are reserved, the resulting traversability
     * class is in [2, * 2 + class_count[
     *
     * If one of the modality is missing, it is simply ignored
     */
    class SimpleTraversability : public envire::Operator {
        ENVIRONMENT_ITEM( SimpleTraversability );

        enum INPUT_DATA {
            SLOPE,
            MAX_STEP,
            MAX_FORCE,
            INPUT_COUNT
        };

        enum CLASSES {
            CLASS_UNKNOWN = 0,
            CLASS_OBSTACLE = 1,
            CUSTOM_CLASSES = 2
        };

        /** Our input layers. This is stored here in addition to store it in the
         * operator graph, as we might use multiple bands of the same layer
         *
         * They are store as integers to support deserialization properly (we
         * have no way in unserialize() to get hold on our input layers)
         */
        std::string input_layers_id[INPUT_COUNT];
        /** The bands that should be used in the input layers */
        std::string input_bands[INPUT_COUNT];

        envire::Grid<float>* getInputLayer(INPUT_DATA index) const;
        std::string getInputBand(INPUT_DATA index) const;

        // Note: no need to store the output layer as it is accessible from the
        // operation graph
        std::string output_band;

        SimpleTraversabilityConfig conf;

    public:
        typedef envire::Grid<uint8_t> OutputLayer;

        SimpleTraversability();
        SimpleTraversability(SimpleTraversabilityConfig const& conf);
        SimpleTraversability(
                double weight_force,
                double force_threshold,
                int class_count,
                double min_width,
                double ground_clearance);

        envire::Grid<float>* getSlopeLayer() const;
        std::string getSlopeBand() const;
        void setSlope(envire::Grid<float>* grid, std::string const& band_name);

        envire::Grid<float>* getMaxStepLayer() const;
        std::string getMaxStepBand() const;
        void setMaxStep(envire::Grid<float>* grid, std::string const& band_name);

        envire::Grid<float>* getMaxForceLayer() const;
        std::string getMaxForceBand() const;
        void setMaxForce(envire::Grid<float>* grid, std::string const& band_name);

        std::string getOutputBand() const;
        void setOutput(OutputLayer* grid, std::string const& band_name);

        bool updateAll();
        void closeNarrowPassages(OutputLayer& map, std::string const& band_name, double min_width);

        void serialize(envire::Serialization& so);
        void unserialize(envire::Serialization& so);
    };
}

#endif

