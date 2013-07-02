#ifndef __ENVIRE_ENVIRONMENT_ITEM__
#define __ENVIRE_ENVIRONMENT_ITEM__

#include <envire/core/Transform.hpp>
#include <envire/core/Serialization.hpp>
#include <boost/intrusive_ptr.hpp>
#include <string>


#define ENVIRONMENT_ITEM_DEF( _classname ) \
const std::string _classname::className = "envire::" #_classname; \
static envire::SerializationPlugin<_classname> _classname ## factory;

#define ENVIRONMENT_ITEM( _classname ) \
        public:\
        static const std::string className; \
        const std::string& getClassName() const { return className; } \
        void set( EnvironmentItem* other ) \
        {\
            _classname* fn = dynamic_cast<_classname*>( other ); \
            if( fn ) \
            {\
                this->operator=( *fn );\
            }\
        }\
        typedef boost::intrusive_ptr<_classname> Ptr; \
        _classname* clone() const \
        { \
            return new _classname( *this );\
        }\
        private:\

namespace envire
{
    class Environment;
    class Serialization;
    class FrameNode;
    class Layer;
    class Operator;
    class CartesianMap;
    class EventHandler;
    class SynchronizationEventQueue;
    class Event;
    class SerializationFactory;


    /** Base class for alle items that are defined in the envire framework.
     * Mainly handles the unique_id feature and the pointer to the environment
     * object.
     *
     * The unique_id of an item is a string representation with an optional
     * integer part. Ids given on construction of an EnvironmentItem based
     * object are first prefixed by the environment prefix (which defaults to
     * '/'), then the provided string is used to form an unique identifier. If
     * this identifier is already in the environment, attaching it will throw.
     * There is however the option to add a trailing '/' to the id. In this
     * case, attaching the item in this case will not generate an exception,
     * but will make the item unique by appending a number to make the complete
     * id unique.
     *
     * [/<environment_prefix>]/<id>[/<numeric_id]
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
	std::string unique_id;

	/** non-unique label which can be used for any purpose
	 */
	std::string label;
	
	/** store pointer to environment to allow convenience methods
	 * referencing the environment object the item is attached to.
	 */
	Environment* env;

    public:
	static const std::string className;
	
	explicit EnvironmentItem(std::string const& id);

	/** overide copy constructor, to allow copying, but remove environment
	 * information for the copied item */
	EnvironmentItem(const EnvironmentItem& item);
	EnvironmentItem& operator=(const EnvironmentItem& other);

	/** Creates a clone of this item. 
	 */
        virtual EnvironmentItem* clone() const { 
	    throw std::runtime_error("clone() not implemented. Did you forget to use the ENVIRONMENT_ITEM macro?."); }

	/** virtual assignemt of other value to this
	 */
	virtual void set( EnvironmentItem* other ) { 
	    throw std::runtime_error("set() not implemented. Did you forget to use the ENVIRONMENT_ITEM macro?."); }

	/** will attach the newly created object to the given Environment.
	 */ 
	explicit EnvironmentItem(Environment* env);	

	virtual ~EnvironmentItem();

	virtual void serialize(Serialization &so);
        
        virtual void unserialize(Serialization &so);

	virtual const std::string& getClassName() const {return className;};

	/** @return the environment this object is associated with 
	 */
	Environment* getEnvironment() const;

	/** @return true if attached to an environment
	 */	
	bool isAttached() const;

	/** Sets all or part of the uniqe ID for this item.
         *
         * This is made unique by the environment when this item is attached to
         * an environment. This method will raise logic_error if used after the
         * item has been attached.
	 */
	void setUniqueId(std::string const& id);
        
	/** @return the unique id of this environmentitem
	 */
	std::string getUniqueId() const;
        
        /** @return the environment prefix, which is part of the unique id
         */
        std::string getUniqueIdPrefix() const;
        
        /** @return the suffix (last part after the /) of the unique id
         */
        std::string getUniqueIdSuffix() const;

	/** @return the suffix of the unique id and perform a conversion to
	 * integer type
	 *
	 * will throw if suffix is not actually numerical, which happens, when
	 * the original unique id given had a trailing slash.
         */
        long getUniqueIdNumericalSuffix() const;

	/** marks this item as modified
	 */
	void itemModified();

	/** will detach the item from the current environment
	 */
	EnvironmentItem::Ptr detach();

	/** get the non-unique label attached to this item
	 */
	std::string getLabel() const { return label; }

	/** set the non-unique label attached to this item
	 */
	void setLabel( const std::string& label ) { this->label = label; }

	/** @return how many objects have a reference on this item
	 */
	long getRefCount() const { return ref_count; }
    };

    void intrusive_ptr_add_ref( EnvironmentItem* item );
    void intrusive_ptr_release( EnvironmentItem* item );
    


}
#endif