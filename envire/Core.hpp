#ifndef __ENVMAP_H__
#define __ENVMAP_H__

#include <list>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

// TODO: use shared_ptr if it is the decided way
//   - update documentation to explain memory management
namespace envire
{
    class Layer;
    class CartesianMap;
    class FrameNode;
    class Operator;

    typedef boost::shared_ptr<Layer> Layer_Ptr;
    typedef boost::shared_ptr<const Layer> Layer_ConstPtr;
    typedef boost::shared_ptr<CartesianMap> CartesianMap_Ptr;
    typedef boost::shared_ptr<const CartesianMap> CartesianMap_ConstPtr;
    typedef boost::shared_ptr<FrameNode> FrameNode_Ptr;
    typedef boost::shared_ptr<const FrameNode> FrameNode_ConstPtr;
    typedef boost::shared_ptr<Operator> Operator_Ptr;
    typedef boost::shared_ptr<const Operator> Operator_ConstPtr;

    /** A 3D transformation 
     * Uses Eigen2 types for rotation and translation.  Internally the Frame
     * is represented as translation and rotation separated, and not in a 4x4
     * Transformation matrix.
     */
    struct Frame
    {
        Frame(const Eigen::Vector3f& translation = Eigen::Vector3f::Zero(),
                const Eigen::Quaternionf& rotation = Eigen::Quaternionf() )
            : translation(translation), rotation(rotation) {}

        Eigen::Vector3f    translation;
        Eigen::Quaternionf rotation;
    };

    /** A node in the frame tree. It represents a single frame of reference, and
     * contains a list of maps that represent information in this frame of reference.
     */
    class FrameNode
    {
        /** The parent frame, or NULL for the root frame */
        FrameNode_Ptr parent;
        /** The 3D transformation that leads from the parent frame to this one
        */
        Frame frame;
        /** The list of maps that are expressed in this frame */
        std::list<CartesianMap_Ptr> maps;

    public:
        /** constructs a new FrameNode Object with the @param node as parent
         * node
         */
        FrameNode( FrameNode_Ptr parent );

        /** default constructor */
        FrameNode();

        /** Returns true if this frame is the root frame (i.e. has no parent) */
        bool isRoot() const;
        /** Returns the frame that is parent of this one, or raises
         * std::runtime_error if it is a root frame
         */
        FrameNode_ConstPtr getParent() const;
        /** Returns the frame that is parent of this one, or raises
         * std::runtime_error if it is a root frame
         */
        FrameNode_Ptr getParent();
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
    
    /** An environment representation is basically a set of Layer. Moreover, one
     * specific type of layers, CartesianMap layers, are managed in a frame
     * tree where relative 3D transformations between the CartesianMap are
     * managed
     */
    class Environment
    {
        /** The set of layers available on this environment
         */
        std::list<Layer_Ptr>    layers;

        /** The definition of the global frame of this environment, represented
         * w.r.t. a "universal frame". That universal frame can for instance be
         * the local UTM coordinates.
         *
         * From there, one can access the whole frame tree
         */
        FrameNode_Ptr frame_tree;

    public:
        Environment();
        
        /** Returns the transformation from the frame represented by @a from to
         * the frame represented by @a to. This always defines an unique
         * transformation, as the frames are sorted in a tree
         */
        Frame relativeTransform(FrameNode const& from, FrameNode const& to);

        /** References a new layer in the environment representation. The
         * ownership of the layer object is passed to the Environment instance.
         * In particular, the layer will be deleted when the environment
         * representation is.
         *
         * @see removeLayer
         */
        void addLayer(Layer_Ptr layer);

        /** De-references this layer from the environment representation.
         * This deletes the layer instance.
         *
         * @see addLayer
         */
        void removeLayer(Layer_Ptr layer);

        /** @return a reference to the root FrameNode of the Environment
         */
        FrameNode_Ptr getRootNode();

        /** will import the scene file specified by @param file
         * uses @param node as the parent FrameNode where the scene is
         * attached.
         */
        bool loadSceneFile( const std::string& file, FrameNode_Ptr node );
        
        /** imports scene and attaches it to the root node
         */
        bool loadSceneFile( const std::string& file);
    };

    /** An operator generates a set of output maps based on a set of input maps
     */
    class Operator
    {
    protected:
        std::list<Layer_Ptr> inputs;
        std::list<Layer_Ptr> outputs;

    public:
        /** Update the output layer(s) according to the defined operation.
         */
        virtual bool updateAll() = 0;

        /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addInput(Layer_Ptr layer);
        /** Removes an input from this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual void removeInput(Layer_Ptr layer);
         /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addOutput(Layer_Ptr layer);
        /** Removes an output from this operator. The operator may not support
         * this, in which case it will return false.
         *
         * You usually don't have to call this yourself:
         * <ul>
         *  <li> if you want to de-reference the given layer, then use Environment::removeLayer()
         *  <li> if you want to detach that layer from the operation (to get the
         *  right to modify it for instance), then use Layer::detachOperator()
         * </ul>
         */
        void removeOutput(Layer_Ptr layer);
    };

    /** The layer is the base object that holds map data. It can be a cartesian
     * map (CartesianMap) or a non-cartesian one (AttributeList, TopologicalMap)
     *
     * The Layer class assumes to be owned by a shared_ptr. 
     */
    class Layer : public boost::enable_shared_from_this<Layer>
    {
        std::string id;
        bool immutable;
        bool dirty; 
        Operator_Ptr generator;

    public:
        /** Create a new layer and adds it to the given environment */
        explicit Layer(std::string const& id = "");
        
        virtual ~Layer();

        /** Returns a string identifier that can be used for debugging purposes
         */
        std::string getID() const;

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
        virtual Layer_Ptr clone(std::string const& id) = 0;

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
        
        /** if this layer is being generated, the operators addOutput will
         * call this method to register the operator. 
         */
        bool setGenerator( Operator_Ptr );
        
        /** Returns the operator that generated this layer, or raises
         * std::runtime_error if it is not a generated map
         */
        Operator_Ptr getGenerator() const;
        /** Recomputes this layer by applying the operator that has already
         * generated this map. The actual operation will only be called if the
         * dirty flag is set, so it is optimal to call it whenever an updated
         * map is needed. After this call, it is guaranteed that isDirty()
         * returns false.
         */
        void updateFromOperator();
    };

    /** This is a special type of layer that describes a map in a cartesian
     * space. These maps have the special feature that they are managed in a
     * frame tree: each map is associated to a given frame (FrameNode). The
     * FrameNode objects themselves forming kinematic chains.
     */
    class CartesianMap : public Layer
    {
    protected:
        FrameNode_Ptr frame;

    public:
        CartesianMap(FrameNode_Ptr node, std::string const& id = "");

        /** Sets the frame node on which this map is attached */
        void setFrameNode(FrameNode_Ptr frame);
        /** Returns the frame node on which this map is attached */
        FrameNode_Ptr getFrameNode();
        /** Returns the frame node on which this map is attached */
        FrameNode_ConstPtr getFrameNode() const;
    };
}

#endif
