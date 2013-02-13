#ifndef ENVIRE_TOOLS_LISTGRID_HPP__
#define ENVIRE_TOOLS_LISTGRID_HPP__

#include <algorithm>
#include <boost/multi_array.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/pool.hpp>

namespace envire
{

template <class C>
class ListGrid
{
    struct Item : public C
    {
	Item* next;
	Item** pthis;
    };

public:
    template <class T>
    class iterator_base : public boost::iterator_facade<
	iterator_base<T>,
	T,
	boost::forward_traversal_tag
	>
    {
	friend class boost::iterator_core_access;
	friend class ListGrid<C>;
	T* m_item;

	iterator_base(T* item) : m_item(item) {}

	void increment() 
	{ 
	    m_item = m_item->next; 
	}
	bool equal( iterator_base<T> const& other ) const 
	{ 
	    return m_item == other.m_item; 
	}
	T& dereference() const 
	{ 
	    return *m_item; 
	}

    public:
	iterator_base<T>() : m_item(NULL) {}

	iterator_base(iterator_base<T> const& other)
	    : m_item(other.m_item) {}
    };

    typedef iterator_base<Item> iterator;
    typedef iterator_base<const Item> const_iterator;

public:
    ListGrid( size_t sizeX, size_t sizeY )
	: cells( boost::extents[sizeX][sizeY] ),
	mem_pool( sizeof( Item ) )
    {
    }

    /** Returns the iterator on the first registered patch at \c xi and \c
     * yi
     */
    iterator beginCell( size_t xi, size_t yi )
    {
	return iterator( cells[xi][yi] );
    }

    /** Returns the first const iterator on the first registered patch at \c
     * xi and \c yi
     */
    const_iterator beginCell( size_t xi, size_t yi ) const
    {
	return const_iterator( cells[xi][yi] );
    }

    /** Returns the past-the-end iterator for cell iteration */
    iterator endCell()
    {
	return iterator();
    }
    /** Returns the const past-the-end iterator for cell iteration */
    const_iterator endCell() const
    {
	return const_iterator();
    }

    /** Inserts a new surface patch at the beginning of the patch list at
     * the given position
     */
    void insertHead( size_t xi, size_t yi, const C& value )
    {
	Item* n_item = static_cast<Item*>(mem_pool.malloc());
	static_cast<C&>(*n_item).operator=(value);
	n_item->next = cells[xi][yi];
	n_item->pthis = &cells[xi][yi];
	if( n_item->next )
	    n_item->next->pthis = &n_item->next;

	cells[xi][yi] = n_item;
    }
    
    /** Inserts a new surface patch at the end of the patch list at
     * the given position
     */
    void insertTail( size_t xi, size_t yi, const C& value )
    {
	iterator last, it;
	last = it = beginCell( xi, yi );
	while( it != endCell() )
	{
	    last = it;
	    it++;
	}

	Item* n_item = static_cast<Item*>(mem_pool.malloc());
	static_cast<C&>(*n_item).operator=(value);
	n_item->next = NULL;

	if( last != endCell() )
	{
	    last.m_item->next = n_item;
	    n_item->pthis = &last.m_item->next;
	}
	else
	{
	    cells[xi][yi] = n_item;
	    n_item->pthis = &cells[xi][yi];
	}
    }

    /** Removes the patch pointed-to by \c position */
    iterator erase( iterator position )
    {
	Item* &p( position.m_item );
	iterator res( p->next );

	*p->pthis = p->next;
	if( p->next )
	    p->next->pthis = p->pthis; 

	mem_pool.free( p );

	return res; 
    }

    void clear()
    {
	std::fill( cells.begin(), cells.end(), NULL );
	mem_pool.purge_memory();
    }

protected:
    boost::multi_array<Item*,2> cells; 
    boost::pool<> mem_pool;
};

}

#endif
