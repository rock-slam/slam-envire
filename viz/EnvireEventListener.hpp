#ifndef ENVIREEVENTLISTENER_H
#define ENVIREEVENTLISTENER_H

#include <vector>
#include <map>
#include <envire/Core.hpp>
#include <envire/core/EventHandler.hpp>
#include "EnvironmentItemVisualizer.hpp"
#include <osg/Group>

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

namespace vizkit 
{

/** 
 * Implements a front/back buffering method for structural changes to 
 * the scene graph. Since we can only update the scenegraph in the update
 * phase, any costly operations, like creating structures to visualize maps
 * are performed on back nodes, that are not linked to the scenegraph.
 *
 * Special handling has to be performed for child nodes, since the nodes of the
 * visualizers can also have child nodes. To differentiate between children of
 * the visualization plugins and of the scene nodes, they are store separately.
 *
 * A call to update() will take the current state of the referenced Environment
 * item and call the referenced visualizer to perform an update of that
 * particular back node. Update needs to be called in the same thread as the
 * environment is changed.
 *
 * Any changes to the back node will be visible after a call to apply(). This
 * method needs to be called in the update phase of osg.
 */
class EnvireNode : public osg::Group
{
    struct Buffer 
    {
	Buffer() : dirty(false) {}
	void swap() { std::swap( front, back ); }
	osg::ref_ptr<osg::Group> front, back;
	bool dirty;
    };

    envire::EnvironmentItem *item;
    EnvironmentItemVisualizer *viz;

    boost::try_mutex mu;
    Buffer node, children;

public:
    EnvireNode( envire::EnvironmentItem *item, EnvironmentItemVisualizer *viz );
    void addChildNode( osg::Group* child );
    void removeChildNode( osg::Group* child );
    void update();
    bool isDirty();
    void apply();
};

class EnvireEventListener : public envire::EventListener
{
    typedef boost::function<void (osg::Node*)> nodeCallback;

    public:
	EnvireEventListener(nodeCallback add, nodeCallback remove);

	void addVisualizer(EnvironmentItemVisualizer *viz);
	void removeVisualizer(EnvironmentItemVisualizer *viz);
	
	osg::Group *getNodeForItem(envire::EnvironmentItem* item);
	EnvironmentItemVisualizer *getVisualizerForItem(envire::EnvironmentItem* item);
	
	virtual void childAdded( envire::FrameNode* parent, envire::FrameNode* child );    
	virtual void childRemoved( envire::FrameNode* parent, envire::FrameNode* child );
	virtual void childAdded( envire::Layer* parent, envire::Layer* child );
	virtual void childRemoved( envire::Layer* parent, envire::Layer* child );
	virtual void itemModified( envire::EnvironmentItem* item );
	virtual void itemAttached( envire::EnvironmentItem* item );
	virtual void itemDetached( envire::EnvironmentItem* item );
	virtual void frameNodeSet( envire::CartesianMap* map, envire::FrameNode* node);
	virtual void frameNodeDetached(envire::CartesianMap* map, envire::FrameNode* node);
	virtual void setRootNode(envire::FrameNode* root);
	virtual void removeRootNode(envire::FrameNode* root);
	
	void apply();

    protected:
	boost::mutex mu;

	void addChild( envire::EnvironmentItem* parent, envire::EnvironmentItem* child );    
	void removeChild( envire::EnvironmentItem* parent, envire::EnvironmentItem* child );
	
	std::vector<EnvironmentItemVisualizer*> visualizers;
	std::map<envire::EnvironmentItem*, osg::ref_ptr<EnvireNode> > environmentToNode;

	nodeCallback osgAddNode, osgRemoveNode;
};

}

#endif // ENVIREEVENTLISTENER_H
