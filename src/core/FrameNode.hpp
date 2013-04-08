#ifndef __ENVIRE_FRAMENODE__
#define __ENVIRE_FRAMENODE__

#include "Transform.hpp"
#include "EnvironmentItem.hpp"

namespace envire
{
    /** An object of this class represents a node in the FrameTree. The
     * FrameTree has one root node (call to env->getRootNode()), which
     * represents the global frame. Each child defines a new frame of reference,
     * that is connected to its parent frame through the transformation object.
     *
     * The transformation C^P_C associated with this object will transform from 
     * this FrameNodes Frame into the parent Frame.
     *
     * A FrameNode holds a TransformWithUncertainty object representing the
     * transformation from child to parent node. This transformation may have
     * uncertainty associated with it, but doesn't have to. In the latter case,
     * a fast path for calculation of transformation chains can be used.
     */
    class FrameNode : public EnvironmentItem
    {
	ENVIRONMENT_ITEM( FrameNode )

    public:
        typedef Transform TransformType;
        
	/** class needs to be 16byte aligned for Eigen vectorisation */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** default constructor */
        FrameNode();
        explicit FrameNode(const Transform &t);
        explicit FrameNode(const TransformWithUncertainty &t);

	virtual void serialize(Serialization &so);
        virtual void unserialize(Serialization &so);

        /** Returns true if this frame is the root frame (i.e. has no parent) */
        bool isRoot() const;

        /** Returns the frame that is parent of this one, or raises
         * @throw std::runtime_error if it is a root frame
         */
        const FrameNode* getParent() const;

        /** Returns the frame that is parent of this one, or raises
         */
        FrameNode* getParent();

        /** Returns the frame that is the root of the frame tree in which this
         * frame itself is stored
         */
        const FrameNode* getRoot() const;

        /** Returns the frame that is the root of the frame tree in which this
         * frame is stored
         */
        FrameNode* getRoot();

	/** Will add the @param child to the current list of children, if this
	 * item is attached.
	 */
	void addChild( FrameNode *child );

        /** Returns the transformation from this FrameNode to the parent framenode
         */
	Transform const& getTransform() const;

        /** Updates the transformation between that node and its parent.
         * Relevant operators will be notified of that change, and all data that
         * has been generated based on that information will be marked as dirty
         */
        void setTransform(Transform const& transform);

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
	Transform relativeTransform( const FrameNode* to ) const;

	/** 
	 * @return a list of maps attached to this framenode.
	 */
	std::list<CartesianMap*> getMaps(); 

	/** 
	 * @return the children of this framenode
	 */
	std::list<FrameNode*> getChildren();

    protected:
	TransformWithUncertainty frame;
    };
}
#endif