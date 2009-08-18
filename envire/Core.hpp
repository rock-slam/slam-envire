#ifndef __ENVMAP_H__
#define __ENVMAP_H__

#include <list>
#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace envire
{
    class Layer;
    class CartesianMap;
    class FrameNode;
    class Operator;
    class Environment;

    /** A 3D transformation 
     * Uses Eigen2 types for rotation and translation.  Internally the Frame
     * is represented as translation and rotation separated, and not in a 4x4
     * Transformation matrix.
     */
    class Frame
    {
    protected:
        Eigen::Vector3f translation;
        Eigen::Quaternionf rotation;

    public:
        Frame(const Eigen::Vector3f& translation = Eigen::Vector3f::Zero(),
                const Eigen::Quaternionf& rotation = Eigen::Quaternionf() )
            : translation(translation), rotation(rotation) {}

        Eigen::Vector3f& getTranslation() { return translation; }
        const Eigen::Vector3f& getTranslation() const { return translation; }
        
        Eigen::Quaternionf& getRotation() { return rotation; }
        const Eigen::Quaternionf& getRotation() const { return rotation; }

        //TODO: add conversion from and to transformation types. 
    };

    class EnvironmentItem 
    {
    protected:
	friend class Environment;

	/** we track the last id given for assigning new id's
	 */
	static long last_id;

	/** each environment item must have a unique id.
	 */
	long unique_id;
	
	/** store pointer to environment to allow convenience methods
	 */
	Environment* env;

    public:
	EnvironmentItem();	
	explicit EnvironmentItem(Environment* env);	
	virtual ~EnvironmentItem();

	/** return the environment this object is associated with 
	 */
	Environment* getEnvironment();	

	/** returns true if attached to an environment
	 */	
	bool isAttached() const;

	/** returns the unique id of this environmentitem
	 */
	long getUniqueId() const;
    };


    /** A node in the frame tree. It represents a single frame of reference.
     */
    class FrameNode : public EnvironmentItem
    {
    protected:
        /** The 3D transformation that leads from the parent frame to this one
        */
        Frame frame;

    public:
        /** default constructor */
        FrameNode();

        /** Returns true if this frame is the root frame (i.e. has no parent) */
        bool isRoot() const;

        /** Returns the frame that is parent of this one, or raises
         * std::runtime_error if it is a root frame
         */
        const FrameNode* getParent() const;

        /** Returns the frame that is parent of this one, or raises
         * std::runtime_error if it is a root frame
         */
        FrameNode* getParent();

        /** Returns the Transformation that leads from the parent frame to
         * this one
         * std::runtime_error if it is a root frame
         */
        Frame const& getTransform() const;

        /** Returns the Transformation that leads from the parent frame to
         * this one
         * std::runtime_error if it is a root frame
         */
        Frame& getTransform();

        /** Updates the transformation between that node and its parent.
         * Relevant operators will be notified of that change, and all data that
         * has been generated based on that information will be marked as dirty
         */
        void setTransform(Frame const& transform);

    };
    
    /** The layer is the base object that holds map data. It can be a cartesian
     * map (CartesianMap) or a non-cartesian one (AttributeList, TopologicalMap)
     */
    class Layer : public EnvironmentItem
    {
    protected:
        /** the id of the layer. This is a non-unique identifier which can 
	 * be used for easy identification of the layer
	 */ 
        std::string name;

        /** TODO: explain immutability for layer */
        bool immutable;

        /** TODO: explain dirty for a layer */
        bool dirty; 

    public:
	Layer();
	Layer(std::string const& name);
	virtual ~Layer();

        /** Returns a string identifier that can be used for debugging purposes
         */
        std::string getName() const;

        /** True if this layer cannot be changed by any means */
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

	void addChild(Layer* child);
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

    /** An operator generates a set of output maps based on a set of input maps
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

    /** The environment class manages EnvironmentItem objects and has ownership of these. 
     * all dependencies between the objects are handled in the environment class, and convenience
     * methods of the individual objects are available to simplify usage.
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
        Frame relativeTransform(const FrameNode* from, const FrameNode* to);

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
