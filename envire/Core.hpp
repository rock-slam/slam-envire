#ifndef __ENVIRE_H__
#define __ENVIRE_H__

#include <list>
#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <boost/serialization/singleton.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/intrusive_ptr.hpp>
#include <vector>
#include <stdexcept>

#include <envire/core/EventSource.hpp>

#define ENVIRONMENT_ITEM_DEF( _classname ) \
const std::string& _classname::className = "envire::" #_classname; \
_classname::Factory _classname::factory( _classname::className  );

#define ENVIRONMENT_ITEM( _classname ) \
	struct Factory { \
	    Factory( const std::string& className ) { \
		SerializationFactory::addClass( className, &createItem<_classname> ); \
	    }\
	}; \
	static Factory factory;\
	public:\
	static const std::string& className; \
	const std::string& getClassName() const { return className; } \
	void set( EnvironmentItem* other ) \
	{\
	    _classname* fn = dynamic_cast<_classname*>( other ); \
	    if( fn ) \
	    {\
		_classname* t = dynamic_cast<_classname*>( this ); \
		t->operator=( *fn );\
	    }\
	}\
	typedef boost::intrusive_ptr<_classname> Ptr; \
	_classname* clone() const \
	{ \
	    const _classname* fn = dynamic_cast<const _classname*>( this ); \
	    if( fn )\
		return new _classname( *fn );\
	    else \
		return NULL;\
	}\
	protected:\

namespace envire
{
    class Layer;
    class CartesianMap;
    class FrameNode;
    class Operator;
    class Environment;
    class EnvironmentItem;
    class Serialization;
    class EventHandler;
    class Event;
    class SerializationFactory;

    /** Baseclass for generically holding pointer to objects, while still
     * ensuring, that the destructor of that object is called, when the holder
     * object is destructed.
     */
    class HolderBase
    {
    public:
	virtual ~HolderBase() {};

        template <typename T> bool isOfType() const;
	template <typename T> T& get();
    	template <typename T> const T& get() const;
    };

    /** Templated holder class, that will construct an object of type T,
     * provide access to it, and also delete the object again when it is
     * destroyed.
     */
    template <typename T>
	class Holder : public HolderBase 
    {
	T* ptr;

    public:
	Holder()
	{
	    ptr = new T();
	};

	~Holder()
	{
	    delete ptr;
	};

	T* getData() const
	{
	    return ptr;
	}
    };

    template <typename T> bool HolderBase::isOfType() const
    {
        Holder<T> const* myself = dynamic_cast< Holder<T> const* >(this);
        return myself ? true : false;
    }
    template <typename T> T& HolderBase::get()
    {
        return *dynamic_cast< Holder<T>* >(this)->getData();
    }

    template <typename T> 
    const T& HolderBase::get() const
    {
        return *dynamic_cast< Holder<T>* >(this)->getData();
    }

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
    public:
	typedef boost::intrusive_ptr<EnvironmentItem> Ptr; 

    protected:
	friend class Environment;
	friend class Event;
	friend void intrusive_ptr_add_ref( EnvironmentItem* item );
	friend void intrusive_ptr_release( EnvironmentItem* item );

	long ref_count;

	/** each environment item must have a unique id.
	 */
	long unique_id;
	
	/** store pointer to environment to allow convenience methods
	 * referencing the environment object the item is attached to.
	 */
	Environment* env;

    public:
	static const std::string className;
	
	EnvironmentItem();	
	explicit EnvironmentItem(Serialization &so);	

	/** overide copy constructor, to allow copying, but remove environment
	 * information for the copied item */
	EnvironmentItem(const EnvironmentItem& item);
	EnvironmentItem& operator=(const EnvironmentItem& other);

	/** Creates a clone of this item. 
	 */
        virtual EnvironmentItem* clone() const { throw std::runtime_error("clone() not implemented. Did you forget to use the ENVIRONMENT_ITEM macro?."); }

	/** virtual assignemt of other value to this
	 */
	virtual void set( EnvironmentItem* other ) { throw std::runtime_error("set() not implemented. Did you forget to use the ENVIRONMENT_ITEM macro?."); }

	/** will attach the newly created object to the given Environment.
	 */ 
	explicit EnvironmentItem(Environment* env);	

	virtual ~EnvironmentItem();

	virtual void serialize(Serialization &so);

	virtual const std::string& getClassName() const {return className;};

	/** @return the environment this object is associated with 
	 */
	Environment* getEnvironment() const;

	/** @return true if attached to an environment
	 */	
	bool isAttached() const;

	/** @return the unique id of this environmentitem
	 */
	long getUniqueId() const;

	/** marks this item as modified
	 */
	void itemModified();

	/** will detach the item from the current environment
	 */
	EnvironmentItem::Ptr detach();
    };

    void intrusive_ptr_add_ref( EnvironmentItem* item );
    void intrusive_ptr_release( EnvironmentItem* item );
    
    typedef Eigen::Vector3d Point;
    
    template<class T> EnvironmentItem* createItem(Serialization &so) 
    {
	T* o = new T(so);
	return o;
    }

    class SerializationFactory 
    {
    public:
	/** Factory typedef, which needs to be implemented by all EnvironmentItems
	 * that are serialized.
	 */
	typedef EnvironmentItem* (*Factory)(Serialization &);

	/** Stores the mapping for all classes that can be serialized, and a function
	 * pointer to the Factory method of that class.
	 */
	static std::map<std::string, Factory>& getMap();

	/** create and object for the given class. Will throw if no such class is registered.
	 */
	static EnvironmentItem* createObject( const std::string& className, Serialization& so );

	/** register a class with the factory
	 */
	static void addClass( const std::string& className, Factory f );
    };

    class PointWithUncertainty
    {
    public:
	typedef Eigen::Matrix<double,3,3> Covariance;

    public:
	PointWithUncertainty();
	PointWithUncertainty( const Point& point, const Covariance& cov );

	const Covariance& getCovariance() const { return cov; }
	void setCovariance( const Covariance& cov ) { this->cov = cov; }
	const Point& getPoint() const { return point; }
	void setPoint( const Point& point ) { this->point = point; }

    protected:
	Point point;
	Covariance cov;
    };

    typedef Eigen::Transform3d Transform;

    class TransformWithUncertainty
    {
    public:
	typedef Eigen::Matrix<double,6,6> Covariance;

    public:
	TransformWithUncertainty();
	explicit TransformWithUncertainty( const Transform& trans );
	TransformWithUncertainty( const Transform& trans, const Covariance& cov );

	TransformWithUncertainty operator*( const TransformWithUncertainty& trans ) const;
	PointWithUncertainty operator*( const PointWithUncertainty& point ) const;
	TransformWithUncertainty inverse( Eigen::TransformTraits traits = Eigen::Isometry ) const;

	const Covariance& getCovariance() const { return cov; }
	void setCovariance( const Covariance& cov ) { this->cov = cov; uncertain = true; }
	const Transform& getTransform() const { return trans; }
	void setTransform( const Transform& trans ) { this->trans = trans; }

	bool hasUncertainty() const { return uncertain; }

    protected:
	Transform trans;
	Covariance cov;
	bool uncertain;
    };

    /** An object of this class represents a node in the FrameTree. The
     * FrameTree has one root node (call to env->getRootNode()), which
     * represents the global frame. Each child defines a new frame of reference,
     * that is connected to its parent frame through the transformation object.
     *
     * The transformation C^P_C associated with this object will transform from 
     * this FrameNodes Frame into the parent Frame.
     */
    class FrameNode : public EnvironmentItem
    {
	ENVIRONMENT_ITEM( FrameNode )

    public:
	typedef Transform TransformType;

    public:
	/** class needs to be 16byte aligned for Eigen vectorisation */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** default constructor */
        FrameNode();
        FrameNode(const TransformType &t);
        FrameNode(Serialization &so);

	virtual void serialize(Serialization &so);

        /** Returns true if this frame is the root frame (i.e. has no parent) */
        bool isRoot() const;

        /** Returns the frame that is parent of this one, or raises
         * @throw std::runtime_error if it is a root frame
         */
        const FrameNode* getParent() const;

        /** Returns the frame that is parent of this one, or raises
         */
        FrameNode* getParent();

	/** Will add the @param child to the current list of children, if this
	 * item is attached.
	 */
	void addChild( FrameNode *child );

        /** Returns the transformation from this FrameNode to the parent framenode
         */
	TransformType const& getTransform() const;

        /** Updates the transformation between that node and its parent.
         * Relevant operators will be notified of that change, and all data that
         * has been generated based on that information will be marked as dirty
         */
        void setTransform(TransformType const& transform);

	/** @return the transformation from this FrameNode to the parent
	 * FrameNode with attached uncertainty.
	 */
	TransformWithUncertainty const& getTransformWithUncertainty() const;

	/** overloaded setTransform() which also sets the uncertainty associated
	 * with a transformation.
	 */
	void setTransform(TransformWithUncertainty const& transform);

        /** 
	 * @return the transformation from this frame to
         * the frame represented by @a to. This always defines an unique
         * transformation, as the frames are sorted in a tree.
	 * This is a convenince access to Environment::relativeTransform()
         */
	TransformType relativeTransform( const FrameNode* to ) const;

	/** 
	 * @return a list of maps attached to this framenode.
	 */
	std::list<CartesianMap*> getMaps(); 

    protected:
	TransformWithUncertainty frame;
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

        typedef std::map<std::string, HolderBase*> DataMap;

	/** associating key values with metadata stored in holder objects */ 
	DataMap data_map;

    public:
	static const std::string className;

	Layer();
	virtual ~Layer();

	Layer(Serialization& so);
	void serialize(Serialization& so);

	virtual const std::string& getClassName() const {return className;};

	/** @return a string identifier that can be used for debugging purposes
         */
        std::string getName() const;

        /** @return True if this layer cannot be changed by any means */
        bool isImmutable() const;

        /** Marks this frame as being immutable. This cannot be changed
         * afterwards, as some operators will depend on it.
         */
        void setImmutable();

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
	std::list<Layer*> getParents();

	/** @return for a given path, it will return a suggestion for a filename 
	 * to use when making this layer persistant
	 */
	const std::string getMapFileName( const std::string& path ) const;

	/** will return true if an entry for metadata for the given key exists
	 */
	bool hasData(const std::string& type);

        /** Will return true if a metadata exists for the given key, and this
         * metadata is of the specified type
         */
        template<typename T>
        bool hasData(const std::string& type)
        { return hasData(type) && data_map[type]->isOfType<T>(); }

	/** For a given key, return the metadata associated with it. If the data
	 * does not exist, it will be created.
	 * Will throw a runtime error if the datatypes don't match.
	 */
	template <typename T>
	    T& getData(const std::string& type)
	{
	    if( !hasData( type ) )
	    {
		data_map[type] = new Holder<T>;
	    }

	    /*
	    if( typeid(*data_map[type]) != typeid(Holder<T>) )
	    {
		std::cerr 
		    << "type mismatch. type should be " 
		    << typeid(data_map[type]).name() 
		    << " but is " 
		    << typeid(Holder<T>).name()
		    << std::endl;
		throw std::runtime_error("data type mismatch.");
	    }
	    */

	    return data_map[type]->get<T>();
	};
	
	/** 
	* For a given key, return the metadata associated with it. 
	* If the data does not exist, a std::out_of_range is thrown.
	*/
	template <typename T>
	const T& getData(const std::string& type) const
	{
	    return data_map.at(type)->get<T>();
	    /*
	    if( typeid(*data_map[type]) != typeid(Holder<T>) )
	    {
		std::cerr 
		    << "type mismatch. type should be " 
		    << typeid(data_map[type]).name() 
		    << " but is " 
		    << typeid(Holder<T>).name()
		    << std::endl;
		throw std::runtime_error("data type mismatch.");
	    }
	    */
	};

    };


    /** This is a special type of layer that describes a map in a cartesian
     * space. These maps have the special feature that they are managed in a
     * frame tree: each map is associated to a given frame (FrameNode). The
     * FrameNode objects themselves forming kinematic chains.
     */
    class CartesianMap : public Layer
    {
    public:
	static const std::string className;

	CartesianMap();
        CartesianMap(Serialization& so);

	virtual const std::string& getClassName() const {return className;};

        /** Sets the frame node on which this map is attached */
        void setFrameNode(FrameNode* frame);

        /** Returns the frame node on which this map is attached */
        FrameNode* getFrameNode();

        /** Returns the frame node on which this map is attached */
        const FrameNode* getFrameNode() const;

	/** @return the dimension of the cartesian space (2 or 3) */
	virtual int getDimension() const = 0;

    };

    template <int _DIMENSION>
	class Map : public CartesianMap
    {
	static const int DIMENSION = _DIMENSION;

    public:
	Map() {};
        Map(Serialization& so) : CartesianMap( so ) {};

	typedef Eigen::AlignedBox<double, DIMENSION> Extents;

	int getDimension() const { return DIMENSION; }
	virtual Extents getExtents() const = 0;
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
     * E.g. raw_data -> [convert to map] -> map -> [merge into global map]
     * with map and [operator]
     */
    class Operator : public EnvironmentItem
    {
    public:
	static const std::string className;

	Operator();
	Operator(Serialization& so);

        /** Update the output layer(s) according to the defined operation.
         */
        virtual bool updateAll(){return false;};

        /** Adds a new input to this operator. The operator may not support
         * this, in which case it will return false
         */
        virtual bool addInput(Layer* layer);

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

        /** Removes an output from this operator.          
	 */
        virtual void removeOutput(Layer* layer);

	/** Removes all outputs connected to this operator
	 */
	void removeOutputs();
    };

    /** The environment class manages EnvironmentItem objects and has ownership
     * of these.  all dependencies between the objects are handled in the
     * environment class, and convenience methods of the individual objects are
     * available to simplify usage.
     */
    class Environment 
    {
	friend class Serialization;
	friend class SerializationImpl;
	friend class GraphViz;

	/** we track the last id given to an item, for assigning new id's.
	 */
	long last_id;
    public:
	static const long ITEM_NOT_ATTACHED = -1;

    protected:
	typedef std::map<long, EnvironmentItem::Ptr > itemListType;
	typedef std::map<FrameNode*, FrameNode*> frameNodeTreeType;
	typedef std::multimap<Layer*, Layer*> layerTreeType;
	typedef std::multimap<Operator*, Layer*> operatorGraphType;
	typedef std::map<CartesianMap*, FrameNode*> cartesianMapGraphType;
	
	itemListType items;
	frameNodeTreeType frameNodeTree;
	layerTreeType layerTree;
	operatorGraphType operatorGraphInput;
	operatorGraphType operatorGraphOutput;
	cartesianMapGraphType cartesianMapGraph;
	
	FrameNode* rootNode;

	EventSource eventHandlers;
	void publishChilds(EventHandler* handler, FrameNode *parent);
	void detachChilds(FrameNode *parent, EventHandler* handler);

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
	EnvironmentItem::Ptr detachItem(EnvironmentItem* item);
	
	/**
	* This method will be called by any EnvironmentItem, which was
	* modified. Calling this method will invoke all listeners 
	* attached to the environment and call their itemModified
	* method.
	**/
	void itemModified(EnvironmentItem* item);
	
	EnvironmentItem::Ptr getItem(int uniqueId) const
	{
            itemListType::const_iterator it = items.find(uniqueId);
            if (it == items.end())
                return 0;
            else
                return it->second;
        }

        /** Returns the only item of the given type.
         *
         * This is a convenience method to search for an item (usually, a map)
         * in environments where it is known to be the only item of that type
         *
         * Throws std::runtime_error if there is not exactly one match (i.e.
         * either more than one or none)
         */
	template <class T>
	boost::intrusive_ptr<T> getItem() const
	{
            boost::intrusive_ptr<T> result;
            for (itemListType::const_iterator it = items.begin(); it != items.end(); ++it)
            {
                boost::intrusive_ptr<T> map = boost::dynamic_pointer_cast<T>(it->second);
                if (map)
                {
                    if (result)
                        throw std::runtime_error("multiple maps in this environment are of the specified type");
                    else
                        result = map;
                }
            }
            if (!result)
                throw std::runtime_error("no maps in this environment are of the specified type");
            return result;
        }

	template <class T>
	boost::intrusive_ptr<T> getItem(int uniqueId) const
	{
            itemListType::const_iterator it = items.find(uniqueId);
            if (it == items.end())
                return 0;
            else
                return boost::dynamic_pointer_cast<T>(it->second);
        }

	void addChild(FrameNode* parent, FrameNode* child);
	void addChild(Layer* parent, Layer* child);

	void removeChild(FrameNode* parent, FrameNode* child);
	void removeChild(Layer* parent, Layer* child);

	FrameNode* getParent(FrameNode* node);
	std::list<Layer*> getParents(Layer* layer);

	FrameNode* getRootNode();
	std::list<FrameNode*> getChildren(FrameNode* parent);
	std::list<Layer*> getChildren(Layer* parent);

	void setFrameNode(CartesianMap* map, FrameNode* node);
	void detachFrameNode(CartesianMap* map, FrameNode* node);
	
	FrameNode* getFrameNode(CartesianMap* map);
	std::list<CartesianMap*> getMaps(FrameNode* node);
	
	bool addInput(Operator* op, Layer* input);
	bool addOutput(Operator* op, Layer* output);

	bool removeInput(Operator* op, Layer* input);
	bool removeOutput(Operator* op, Layer* output);

	bool removeInputs(Operator* op);
	bool removeOutputs(Operator* op);

	std::list<Layer*> getInputs(Operator* op);

        /** Returns the only layer that is an input of type LayerT for the given
         * operator
         *
         * LayerT must be a pointer-to-map type:
         * 
         * <code>
         * MLSGrid* mls = env->getInput< MLSGrid* >(mls_slope_operator);
         * </code>
         *
         * If multiple matching layers are found, an exception is raised
         */
        template<typename LayerT>
        LayerT getInput(Operator* op)
        {
            std::list<Layer*> inputs = getInputs(op);
            LayerT result = 0;
            for (std::list<Layer*>::iterator it = inputs.begin(); it != inputs.end(); ++it)
            {
                LayerT layer = dynamic_cast<LayerT>(*it);
                if (layer)
                {
                    if (result)
                        throw std::runtime_error("more than one input layer with the required type found");
                    else
                        result = layer;
                }
            }
            if (!result)
                throw std::runtime_error("cannot find an input layer with the required type");
            return result;
        }

	std::list<Layer*> getOutputs(Operator* op);

        /** Returns the only layer that is an output of type T for the given
         * operator
         *
         * LayerT must be a pointer-to-map type:
         * 
         * <code>
         * Grid<double>* slopes = env->getOutput< Grid<double>* >(mls_slope_operator);
         * </code>
         *
         * If multiple matching layers are found, an exception is raised
         */
        template<typename LayerT>
        LayerT getOutput(Operator* op)
        {
            std::list<Layer*> outputs = getOutputs(op);
            LayerT result = 0;
            for (std::list<Layer*>::iterator it = outputs.begin(); it != outputs.end(); ++it)
            {
                LayerT layer = dynamic_cast<LayerT>(*it);
                if (layer)
                {
                    if (result)
                        throw std::runtime_error("more than one output layer with the required type found");
                    else
                        result = layer;
                }
            }
            if (!result)
                throw std::runtime_error("cannot find an output layer with the required type");
            return result;
        }

        /** Returns the operator that has \c output in its output, or NULL if
         * none exist.
         *
         * Note that a layer can be output of a single operator only
         */
	Operator* getGenerator(Layer* output);

        /** Returns the layers that are generated from \c input
         *
         * In practice, it returns the output layers of all the operators for
         * which \c input is listed as input
         */
        std::list<Layer*> getLayersGeneratedFrom(Layer* input);

        /** Returns the layers of type T that are generated from \c input
         *
         * T must be a pointer on a map:
         *
         * <code>
         * env->getGeneratedFrom<Grid<double>*>(mls);
         * </code>
         */
        template<typename T>
        std::list<T> getGeneratedFrom(Layer* input)
        {
            std::list<Layer*> all = getLayersGeneratedFrom(input);
            std::list<T> result;
            for (std::list<Layer*>::const_iterator it = all.begin();
                    it != all.end(); ++it)
            {
                T as_T = dynamic_cast<T>(*it);
                if (as_T) result.push_back(as_T);
            }
            return result;
        }

	void updateOperators();

        /** Serializes this environment to the given directory */
        void serialize(std::string const& path);

        /** Loads the environment from the given directory and returns it */
        static Environment* unserialize(std::string const& path);

	/**
	 * Adds an eventHandler that gets called whenever there
	 * are modifications to the evironment.
	 * On registration, all content of the environment is put to the
	 * handler as if it was being generated.
	 */
	void addEventHandler(EventHandler *handler);

	/**
	 * Remove the given eventHandler from the environment.
	 * The handler will receive events as if the Environment was destroyed.
	 */
	void removeEventHandler(EventHandler *handler);

	/**
	 * will pass the @param event to the registered event handlers
	 */
	void handle( const Event& event );

	/**
	 * returns all items of a particular type
	 */
	template <class T>
	    std::vector<T*> getItems()
	{
	    std::vector<T*> result;
	    for(itemListType::iterator it=items.begin();it != items.end(); ++it )
	    {
		// TODO this is not very efficient...
		T* item = dynamic_cast<T*>( it->second.get() );
		if( item )
		    result.push_back( item );
	    }
	    return result;
	}

        //convenience function to create EnvironmentItems which are automatically attached
        //to the environment 
        template<class T>
        T* create()
        {
            T* p = new T;
            attachItem(p);
            return p;
        }

        /** 
	 * Returns the transformation from the frame represented by @a from to
         * the frame represented by @a to. This always defines an unique
         * transformation, as the frames are sorted in a tree.
	 *
	 * relativeTransform( child, child->getParent() ) is equivalent to
	 * child->getTransform().
         */
	FrameNode::TransformType relativeTransform(const FrameNode* from, const FrameNode* to);

	/** @return a new transform object, that specifies the transformation
	 * from the @param from frame to the @param to frame, and will take care of
	 * uncertainty (linearised) on the way.
	 */
	TransformWithUncertainty relativeTransformWithUncertainty(const FrameNode* from, const FrameNode* to);
    };

    class Serialization
    {
	friend class SerializationImpl;
	class SerializationImpl *impl;

	static const std::string STRUCTURE_FILE;

    public:
	Serialization();
	~Serialization();

	void serialize(Environment* env, const std::string &path);
	Environment* unserialize(const std::string &path);

	void setClassName(const std::string &key);

	const std::string getMapPath() const;

	template <class T> void write(const std::string &key, const T& value);
	void write(const std::string &key, const std::string &value);
	void write(const std::string &key, const FrameNode::TransformType &value);

	template <class T> void read(const std::string &key, T& value);
	bool read(const std::string &key, std::string &value);
	bool read(const std::string &key, FrameNode::TransformType &value);
    };

    template <class T> void Serialization::read(const std::string &key, T& value)
    {
	std::string tmp;
	if( read( key, tmp ) )
	    value = boost::lexical_cast<T>(tmp);
    }

    template <class T> void Serialization::write(const std::string &key, const T& value)
    {
	write( key, boost::lexical_cast<std::string>(value) );
    }


}

#endif
