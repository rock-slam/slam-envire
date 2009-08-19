#ifndef __ENVIRE_H__
#define __ENVIRE_H__

#include <list>
#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace envire
{
    class Layer;
    class CartesianMap;
    class FrameNode;
    class Operator;
    class Environment;

    /** Base class for alle items that are defined in the envire framework.
     * Mainly handles the unique_id feature and the pointer to the environment
     * object.
     *
     * Ownership of objects is managed as follows: Objects are owned by the user
     * as long as they are not attached to the environment.
     *
     * Ownership is passed to the Environment object, once the item is attached
     * to the environment in one of the following ways:
     *
     * - explicit: env->attachItem( item )
     * - implicit: using the item as parameter to a call like item2->addChild(
     *   item )
     *
     * When the ownership is passed, it is not allowed to free the memory of the
     * object, this is now done by the Environment on destruction.
     *
     * The ownership of the items can be passed back to the user by calling
     * env->detachItem( item )
     */
    class EnvironmentItem 
    {
    protected:
	friend class Environment;

	/** Factory typedef, which needs to be implemented by all EnvironmentItems
	 * that are serialized.
	 */
	typedef EnvironmentItem* (*Factory)(std::istream &, std::string &);

	/** Stores the mapping for all classes that can be serialized, and a function
	 * pointer to the Factory method of that class.
	 */
	static std::map<std::string, Factory> classMap;

	/** we track the last id given to an item, for assigning new id's.
	 */
	static long last_id;

	/** each environment item must have a unique id.
	 */
	long unique_id;
	
	/** store pointer to environment to allow convenience methods
	 * referencing the environment object the item is attached to.
	 */
	Environment* env;

    public:
	EnvironmentItem();	

	/** will attach the newly created object to the given Environment.
	 */ 
	explicit EnvironmentItem(Environment* env);	

	virtual ~EnvironmentItem();

	/** virtual function for serialisation of the descriptive members of
	 * the objects. This will usually mean anything that is not actual bulk
	 * data. The reason to differentiate is, that bulk data is stored in a
	 * different manner, using the serializeData function.
	 *
	 * @param os[in] output stream in which to write data of the object to be
	 * serialised
	 * @param path[in] path to data directory, which can be used by the 
	 * serialisation method to store bulk data 
	 */
	virtual void serialize(std::ostream &os, std::string &path);

	/** counterpart to the serialize function.
	 * This is a static function, and requires that each class that can be 
	 * serialized to have a static Factory method, which is added into the classMap.
	 */
	static void unserialize(std::istream &is, std::string &path);

	/** @return the environment this object is associated with 
	 */
	Environment* getEnvironment();	

	/** @return true if attached to an environment
	 */	
	bool isAttached() const;

	/** @return the unique id of this environmentitem
	 */
	long getUniqueId() const;
    };


    /** An object of this class represents a node in the FrameTree. The
     * FrameTree has one root node (call to env->getRootNode()), which
     * represents the global frame. Each child defines a new frame of reference,
     * that is connected to its parent frame through the transformation object.
     *
     * So for P being the parent Frame and F the frame associated with the
     * FrameNode, and T the Transformation, we get F = T*P.
     */
    class FrameNode : public EnvironmentItem
    {
    public:
	typedef Eigen::Transform<float,3> TransformType;

    protected:
        /** The 3D transformation that leads from the parent frame to this one
        */
	TransformType frame;

    public:
	/** class needs to be 16byte aligned for Eigen vectorisation */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** default constructor */
        FrameNode();

	virtual void serialize(std::ostream &os, std::string &path);

	/** will create a FrameNode object from a serialisation 
	 * data stream.
	 */
	static FrameNode* create(std::istream &is, std::string &path);

        /** Returns true if this frame is the root frame (i.e. has no parent) */
        bool isRoot() const;

        /** Returns the frame that is parent of this one, or raises
         * @throw std::runtime_error if it is a root frame
         */
        const FrameNode* getParent() const;

        /** Returns the frame that is parent of this one, or raises
         */
        FrameNode* getParent();

        /** Returns the Transformation that leads from the parent frame to
         * this one
         */
	TransformType const& getTransform() const;

        /** Returns the Transformation that leads from the parent frame to
         * this one
         */
        TransformType& getTransform();

        /** Updates the transformation between that node and its parent.
         * Relevant operators will be notified of that change, and all data that
         * has been generated based on that information will be marked as dirty
         */
        void setTransform(TransformType const& transform);
    };
    
    /** The layer is the base object that holds map data. It can be a cartesian
     * map (CartesianMap) or a non-cartesian one (AttributeList, TopologicalMap)
     */
    class Layer : public EnvironmentItem
    {
    protected:
        /** the name of the layer. This is a non-unique identifier which can 
	 * be used for easy identification of the layer
	 */ 
        std::string name;

        /** @todo explain immutability for layer */
        bool immutable;

        /** @todo explain dirty for a layer */
        bool dirty; 

    public:
	Layer();
	Layer(std::string const& name);
	virtual ~Layer();

	/** @return a string identifier that can be used for debugging purposes
         */
        std::string getName() const;

        /** @return True if this layer cannot be changed by any means */
        bool isImmutable() const;

        /** Marks this frame as being immutable. This cannot be changed
         * afterwards, as some operators will depend on it.
         */
        void setImmutable();

        /** Creates a clone of this layer with the given ID. The clone is
         * detached from any operator and is not immutable. It is not added to
         * any environment either.
         */
        virtual Layer* clone() = 0;

        /** Unsets the dirty flag on this layer
         * @see isDirty
         */
        void resetDirty();

        /** Marks this layer as dirty
         * @see isDirty
         */
        void setDirty();

        /** In case this layer is auto-generated, this flag is true when the
         * layer sources have been updated but this layer has not yet been.
         */
        bool isDirty() const;

        /** Detach this layer from the operator that generates it, and returns
         * true if this operation was a success (not all operators support
         * this). After this method returned true, it is guaranteed that
         * isGenerated() returns false
         */
        bool detachFromOperator();

        /** True if this layer has been generated by an operator */
        bool isGenerated() const;
        
        /** Returns the operator that generated this layer, or raises
         * std::runtime_error if it is not a generated map
         */
        Operator* getGenerator() const;

	/** Recomputes this layer by applying the operator that has already
	 * generated this map. The actual operation will only be called if the
	 * dirty flag is set, so it is optimal to call it whenever an updated
	 * map is needed. After this call, it is guaranteed that isDirty()
	 * returns false.
         */
        void updateFromOperator();
	
	/** Layers can have hierarchical relationships. This function will add
	 * a child layer to this object.
	 * @param child - the child layer to add
	 */
	void addChild(Layer* child);

	/** @return the parent of this layer or NULL if the layer has no parent
	 */
        Layer* getParent();
    };

    /** This is a special type of layer that describes a map in a cartesian
     * space. These maps have the special feature that they are managed in a
     * frame tree: each map is associated to a given frame (FrameNode). The
     * FrameNode objects themselves forming kinematic chains.
     */
    class CartesianMap : public Layer
    {
    public:
	CartesianMap();
        CartesianMap(std::string const& name);

        /** Sets the frame node on which this map is attached */
        void setFrameNode(FrameNode* frame);

        /** Returns the frame node on which this map is attached */
        FrameNode* getFrameNode();

        /** Returns the frame node on which this map is attached */
        const FrameNode* getFrameNode() const;
    };

    /** An operator generates a set of output maps based on a set of input maps.
     * Operators are also managed by the Environment and can represent
     * operational releationsships between layers. Through the features of the
     * layer, which holds information about the update state (e.g. dirty), the
     * operators can be executed automatically on data that requires updates. 
     *
     * The links between operators and layers form a graph, which can represent
     * convenient operation chains. 
     *
     * E.g. <raw_data> -> [convert to map] -> <map> -> [merge into global map]
     * with <map> and [operator]
     */
    class Operator : public EnvironmentItem
    {
    public:
        /** Update the output layer(s) according to the defined operation.
         */
        virtual bool updateAll() = 0;

        /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addInput(Layer* layer);

        /** Removes an input from this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual void removeInput(Layer* layer);

         /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addOutput(Layer* layer);

        /** Removes an output from this operator. The operator may not support
         * this, in which case it will return false.
         */
        virtual void removeOutput(Layer* layer);
    };

    /** The environment class manages EnvironmentItem objects and has ownership
     * of these.  all dependencies between the objects are handled in the
     * environment class, and convenience methods of the individual objects are
     * available to simplify usage.
     */
    class Environment
    {
    protected:
	typedef std::list<EnvironmentItem*> itemListType;
	typedef std::map<FrameNode*, FrameNode*> frameNodeTreeType;
	typedef std::map<Layer*, Layer*> layerTreeType;
	typedef std::multimap<Operator*, Layer*> operatorGraphType;
	typedef std::map<CartesianMap*, FrameNode*> cartesianMapGraphType;

	itemListType items;
	frameNodeTreeType frameNodeTree;
	layerTreeType layerTree;
	operatorGraphType operatorGraphInput;
	operatorGraphType operatorGraphOutput;
	cartesianMapGraphType cartesianMapGraph;

	FrameNode* rootNode;

    public:
        Environment();
	virtual ~Environment();
        
	/** attaches an EnvironmentItem and puts it under the control of the Environment.
	 * Object ownership is passed to the environment in this way.
	 */
	void attachItem(EnvironmentItem* item);

	/** detaches an object from the environment. After this, the object is no longer owned by
	 * by the environment. All links to this object from other objects in the environment are
	 * removed.
	 */
	void detachItem(EnvironmentItem* item);

	void addChild(FrameNode* parent, FrameNode* child);
	void addChild(Layer* parent, Layer* child);

	void removeChild(FrameNode* parent, FrameNode* child);
	void removeChild(Layer* parent, Layer* child);

	FrameNode* getParent(FrameNode* node);
	Layer* getParent(Layer* layer);

	FrameNode* getRootNode();
	std::list<FrameNode*> getChildren(FrameNode* parent);
	std::list<Layer*> getChildren(Layer* parent);

	void setFrameNode(CartesianMap* map, FrameNode* node);
	FrameNode* getFrameNode(CartesianMap* map);
	std::list<CartesianMap*> getMaps(FrameNode* node);
	
	bool addInput(Operator* op, Layer* input);
	bool addOutput(Operator* op, Layer* output);

	bool removeInput(Operator* op, Layer* input);
	bool removeOutput(Operator* op, Layer* output);

	std::list<Layer*> getInputs(Operator* op);
	std::list<Layer*> getOutputs(Operator* op);

	Operator* getGenerator(Layer* output);

        /** Returns the transformation from the frame represented by @a from to
         * the frame represented by @a to. This always defines an unique
         * transformation, as the frames are sorted in a tree
         */
	FrameNode::TransformType relativeTransform(const FrameNode* from, const FrameNode* to);

        /** will import the scene file specified by @param file
         * uses @param node as the parent FrameNode where the scene is
         * attached.
         */
        bool loadSceneFile( const std::string& file );

	/** will save the environment into a scene file.
	 */
	bool saveSceneFile( const std::string& file );
    };

}

#endif
