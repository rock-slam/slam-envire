#ifndef __ENVIRE_H__
#define __ENVIRE_H__

#include <list>
#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/intrusive_ptr.hpp>
#include <vector>
#include <stdexcept>


#include <envire/core/Environment.hpp>
#include <envire/core/EventSource.hpp>
#include <envire/core/EventTypes.hpp>
#include <envire/core/Transform.hpp>
#include <envire/core/Holder.hpp>
#include <envire/core/FrameNode.hpp>
#include <envire/core/Layer.hpp>
#include <envire/core/Operator.hpp>
#include <base/samples/RigidBodyState.hpp>


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
    class SynchronizationEventQueue;
    class Event;
    class SerializationFactory;







    /** This is a special type of layer that describes a map in a cartesian
     * space. These maps have the special feature that they are managed in a
     * frame tree: each map is associated to a given frame (FrameNode). The
     * FrameNode objects themselves forming kinematic chains.
     */
    class CartesianMap : public Layer
    {
    public:
	typedef boost::intrusive_ptr<CartesianMap> Ptr; 

    public:
        virtual ~CartesianMap(){};
	static const std::string className;

	explicit CartesianMap(std::string const& id);

	virtual const std::string& getClassName() const {return className;};

        /** Sets the frame node on which this map is attached */
        void setFrameNode(FrameNode* frame);

        /** Returns the frame node on which this map is attached */
        FrameNode* getFrameNode();

        /** Returns the frame node on which this map is attached */
        const FrameNode* getFrameNode() const;

	/** @return the dimension of the cartesian space (2 or 3) */
	virtual int getDimension() const = 0;

        virtual CartesianMap* clone() const { throw std::runtime_error("clone() not implemented. Did you forget to use the ENVIRONMENT_ITEM macro?."); }

        /** Clones this map and the associated frame tree inside the target
         * environment
         */
        void cloneTo(Environment& env) const;
    };

    template <int _DIMENSION>
	class Map : public CartesianMap
    {
	static const int DIMENSION = _DIMENSION;

    public:

        // Defined later as it requires Environment
        Map();
        virtual ~Map(){};

        explicit Map(std::string const& id)
            : CartesianMap(id) {}

	typedef Eigen::AlignedBox<double, DIMENSION> Extents;

	int getDimension() const { return DIMENSION; }

	/** 
	 * @brief provide the bounding box of the content of the layer in local frame coordinates
	 *
	 * @return the bounding box of all the content in the layer
	 */
	virtual Extents getExtents() const = 0;

	/** 
	 * @brief provide the bounding box given in the provided framenode
	 *
	 * This method will take the bounding box from getExtents() and fit the
	 * box into another box in the given reference frame.
	 *
	 * @return bounding box of the layer expressed in the given framenode
	 *  and not in the local frame
	 */
	virtual Extents getExtentsTransformed( FrameNode const& frame ) const
	{
	    Extents res;

	    // rotate the extents to the grid frame, and extend the local grid 
	    // whith the extents corner
	    Eigen::Affine3d pc2grid = getFrameNode()->relativeTransform( &frame );
	    Extents pc_extents = getExtents();
	    std::vector<Eigen::Vector3d> corners;
	    // go through all the permutations to get the corners
	    for( int i=0; i<pow(2,DIMENSION); i++ )
	    {
		Eigen::Vector3d corner;
		for( int j=0; j<DIMENSION; j++ )
		    corner[j] = (i>>j)&1 ? pc_extents.min()[j] : pc_extents.max()[j];
		corners.push_back( corner );
	    }

	    for( std::vector<Eigen::Vector3d>::iterator it = corners.begin(); it != corners.end(); it++ )
		res.extend( (pc2grid * (*it)).head<DIMENSION>() );

	    return res;
	} 

        /** @overload
         *
         * It uses the root frame as the point's frame
         */
        Eigen::Vector3d toMap(Eigen::Vector3d const& point) const
        {
            return toMap(point, *getFrameNode()->getRoot());
        }

        /** Transforms a point from an arbitrary frame to the map's own frame
         *
         * This is valid only for maps of dimension less than 3
         */
        Eigen::Vector3d toMap(Eigen::Vector3d const& point, FrameNode const& frame) const
        {
            return frame.relativeTransform(getFrameNode()) * point;
        }

        /** @overload
         *
         * It uses the root frame as the target frame
         */
        Eigen::Vector3d fromMap(Eigen::Vector3d const& point) const
        {
            return fromMap(point, *getFrameNode()->getRoot());
        }

        /** Transforms a point from the map's own frame to an arbitrary frame
         *
         * This is valid only for maps of dimension less than 3
         */
        Eigen::Vector3d fromMap(Eigen::Vector3d const& point, FrameNode const& frame) const
        {
            return getFrameNode()->relativeTransform(&frame) * point;
        }
    };

    /** This is defined here for backward compatibility reasons. The default
     * constructor for Map should be removed in the long run, as we want to make
     * sure that all map types define a constructor that allows to set the map
     * ID
     */
    template<int _DIM>
    Map<_DIM>::Map()
        : CartesianMap(Environment::ITEM_NOT_ATTACHED) {}
}

#endif
