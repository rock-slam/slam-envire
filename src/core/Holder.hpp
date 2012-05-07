#ifndef ENVIRE_HOLDER__
#define ENVIRE_HOLDER__

#include <boost/noncopyable.hpp>

namespace envire
{
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
	virtual HolderBase* clone() const = 0;
    };

    /** Templated holder class, that will construct an object of type T,
     * provide access to it, and also delete the object again when it is
     * destroyed.
     */
    template <typename T>
	class Holder : public HolderBase, boost::noncopyable 
    {
	T* ptr;

    public:
	Holder()
	{
	    ptr = new T();
	};

	explicit Holder( T* ptr )
	    : ptr( ptr )
	{
	}

	~Holder()
	{
	    delete ptr;
	};

	T* getData() const
	{
	    return ptr;
	}

	Holder<T>* clone() const
	{
	    T* clone = new T(*ptr);
	    return new Holder<T>(clone);
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
}

#endif
