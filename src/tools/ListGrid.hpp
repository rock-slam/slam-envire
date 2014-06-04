#ifndef ENVIRE_TOOLS_LISTGRID_HPP__
#define ENVIRE_TOOLS_LISTGRID_HPP__

#include <algorithm>
#include <boost/multi_array.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/array.hpp>

namespace envire
{

/**
 * Implementation of a grid structure, where each grid element is a list.
 * The class is templated for the element type of the list.
 */
template <class C>
class ListGrid
{
    struct Item : public C
    {
	Item() {};
	explicit Item( const C& c ) : C( c ) {}

	Item* next;
	Item** pthis;
    };

public:
    template <class T, class TV>
    class iterator_base : public boost::iterator_facade<
	iterator_base<T,TV>,
	TV,
	boost::forward_traversal_tag
	>
    {
	friend class boost::iterator_core_access;
	friend class ListGrid<C>;
	T* m_item;

	explicit iterator_base(T* item) : m_item(item) {}

	void increment() 
	{ 
	    m_item = m_item->next; 
	}
	bool equal( iterator_base<T,TV> const& other ) const 
	{ 
	    return m_item == other.m_item; 
	}
	TV& dereference() const 
	{ 
	    return *m_item; 
	}

    public:
	iterator_base<T,TV>() : m_item(NULL) {}

	iterator_base(iterator_base<T,TV> const& other)
	    : m_item(other.m_item) {}
    };

    typedef iterator_base<Item, C> iterator;
    typedef iterator_base<const Item, const C> const_iterator;

public:
    ListGrid(){
    	mem_pool = new boost::object_pool<Item>();
    }

    ListGrid( size_t sizeX, size_t sizeY )
	: cells( boost::extents[sizeX][sizeY] )
    {
    	delete mem_pool;
    }

    ListGrid( const ListGrid<C>& other )
    {
	// use the assignment operator 
	this->operator=( other );
    }

    ListGrid& operator=( const ListGrid<C>& other )
    {
	if( &other != this )
	{
	    // assert same dimension for grids
	    assert( other.cells.num_dimensions() == cells.num_dimensions() );

	    // test if shape of cells is the same
	    // and reshape our cell structure if necessary
	    if( !std::equal( other.cells.shape(), other.cells.shape() + other.cells.num_dimensions(), cells.shape() ) )
	    {
		boost::array<typename ArrayType::index, 2> shape;
		std::copy( other.cells.shape(), other.cells.shape() + 2, shape.begin() );
		cells.resize( shape );
	    }

	    // clear cell array of this
	    clear();

	    // and for each cell perform a copy
	    for(size_t xi=0;xi<cells.shape()[0];xi++)
	    {
		for(size_t yi=0;yi<cells.shape()[1];yi++)
		{
		    cells[xi][yi] = NULL;
		    for( const_iterator it = other.beginCell( xi,yi ); it != other.endCell(); it++ )
			insertTail( xi, yi, *it );
		}
	    }
	}

	return *this;
    }
    
    /**
     * Moves the contents of the grid by 
     * x and y cells. Cells falling of the grid
     * will be discarded. 'New' cells are filled
     * with empty cells.
     * */
    void move(int xd, int yd)
    {
        if(abs(xd) >= cells.shape()[0] || abs(yd) >= cells.shape()[1] )
        {
            clear();
            return;
        }
        
        //copy grid to tempgrid
        ArrayType tmp;
        boost::array<typename ArrayType::index, 2> shape;
        std::copy( cells.shape(), cells.shape() + 2, shape.begin() ); 
        tmp.resize(shape);

        int width = cells.shape()[0];
        int height = cells.shape()[1];

        boost::swap(tmp, cells);

        for(int x = 0; x < width; x++)
        {
            for(int y = 0; y < height; y++)
            {
                const int newX = x + xd;
                const int newY = y + yd;
                if(newX < 0 || newX >= width || newY < 0 || newY >= height )
                {
                    //cell moved off the grid
                    //delete all entries
                    Item *p = tmp[x][y];
                    while(p)
                    {
                        Item *cur = p;
                        p = cur->next;
                        mem_pool->free(cur);
                    }
                }
                else
                {
                    cells[newX][newY] = tmp[x][y];
                }
            }
        }        
    }
    
    /** resize the grid. This will also clear all content
     */
    void resize( size_t sizeX, size_t sizeY )
    {
	clear();
	cells.resize( boost::extents[sizeX][sizeY] );
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
	Item* n_item = mem_pool->malloc();
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

	Item* n_item = mem_pool->malloc();
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

	mem_pool->free(p);

	return res; 
    }

    void clear()
    {
    	std::vector<Item*> tofree;
    	for (Item** item = cells.origin(); item < cells.origin() + cells.num_elements();item++){
			Item *p = *item;
			while(p)
			{
				Item *cur = p;
				p = cur->next;
				tofree.push_back(cur);
			}
			*item = NULL;
    	}
    	delete mem_pool;
    	mem_pool = new boost::object_pool<Item>();
    }

protected:
    typedef boost::multi_array<Item*,2> ArrayType; 
    ArrayType cells;
    boost::object_pool<Item>* mem_pool;
};

}

#endif
