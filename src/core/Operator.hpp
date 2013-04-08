#ifndef __ENVIRE_OPERATOR__
#define __ENVIRE_OPERATOR__

#include "EnvironmentItem.hpp"
#include "Environment.hpp"

namespace envire
{
    /** An operator generates a set of output maps based on a set of input maps.
     * Operators are also managed by the Environment and can represent
     * operational releationsships between layers. Through the features of the
     * layer, which holds information about the update state (e.g. dirty), the
     * operators can be executed automatically on data that requires updates. 
     *
     * The links between operators and layers form a graph, which can represent
     * convenient operation chains. 
     *
     * E.g. raw_data -> [convert to map] -> map -> [merge into global map]
     * with map and [operator]
     */
    class Operator : public EnvironmentItem
    {
    public:
	typedef boost::intrusive_ptr<Operator> Ptr; 

	static const std::string className;

        /** Constructs a new operator
         *
         * @arg inputArity if nonzero, this is the number of inputs that this
         *                 operator requires
         * @arg outputArity if nonzero, this is the number of outputs that this
         *                  operator requires
         */
	explicit Operator(std::string const& id, int inputArity = 0, int outputArity = 0);

        /** Constructs a new operator
         *
         * @arg inputArity if nonzero, this is the number of inputs that this
         *                 operator requires
         * @arg outputArity if nonzero, this is the number of outputs that this
         *                  operator requires
         */
	explicit Operator(int inputArity = 0, int outputArity = 0);

        /** Update the output layer(s) according to the defined operation.
         */
        virtual bool updateAll(){return false;};

        /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addInput(Layer* layer);

        /** Removes all existing inputs to this operator, and replaces them by
         * \c layer
         *
         * This is usually used for operators that accept only one input
         */
        bool setInput(Layer* layer);

        /** Removes an input from this operator. 
	 */ 
        virtual void removeInput(Layer* layer);

	/** Removes all inputs connected to this operator
	 */
	void removeInputs();

         /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addOutput(Layer* layer);

        /** Removes all existing inputs to this operator, and replaces them by
         * \c layer
         *
         * This is usually used for operators that accept only one output
         */
        bool setOutput(Layer* layer);

        /** Removes an output from this operator.          
	 */
        virtual void removeOutput(Layer* layer);

	/** Removes all outputs connected to this operator
	 */
	void removeOutputs();

        /** Returns the only layer that is an output of type T for this operator
         *
         * LayerT must be a pointer-to-map type:
         * 
         * <code>
         * Grid<double>* slopes = mls_slope_operator->getOutput< Grid<double>* >();
         * </code>
         *
         * If multiple matching layers are found, an exception is raised
         */
        template<typename LayerT>
        LayerT getOutput();

        /** Returns the only layer that is an input of type LayerT for this
         * operator
         *
         * LayerT must be a pointer-to-map type:
         * 
         * <code>
         * MLSGrid* mls = mls_slope_operator->getInput< MLSGrid* >();
         * </code>
         *
         * If multiple matching layers are found, an exception is raised
         */
        template<typename LayerT>
        LayerT getInput();

    private:
        int inputArity;
        int outputArity;
    };

   
    template<typename LayerT>
    LayerT Operator::getInput()
    { return getEnvironment()->getInput<LayerT>(this); }

    template<typename LayerT>
    LayerT Operator::getOutput()
    { return getEnvironment()->getOutput<LayerT>(this); }

}

#endif
