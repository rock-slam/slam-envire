#ifndef ENVIREEVENTLISTENER_H
#define ENVIREEVENTLISTENER_H

#include <vector>
#include <map>
#include <envire/Core.hpp>
#include <envire/core/EventHandler.hpp>
#include "EnvironmentItemVisualizer.hpp"
#include <boost/thread/recursive_mutex.hpp>
#include <osg/ref_ptr>

#include <boost/function.hpp>

namespace osg
{
class Group;
class Node;
}

namespace vizkit {

class EnvireEventListener : public envire::EventListener
{
    typedef boost::function<void (osg::Node*)> nodeCallback;

    public:
	EnvireEventListener(nodeCallback add, nodeCallback remove);
	void addVisualizer(EnvironmentItemVisualizer *viz);
	void removeVisualizer(EnvironmentItemVisualizer *viz);
	
	osg::Group *getNodeForItem(envire::EnvironmentItem* item);
	
	EnvironmentItemVisualizer *getVisualizerForItem(envire::EnvironmentItem* item);
	
	virtual void childAdded ( envire::FrameNode* parent, envire::FrameNode* child );    
	virtual void childRemoved ( envire::FrameNode* parent, envire::FrameNode* child );
	virtual void childAdded ( envire::Layer* parent, envire::Layer* child );
	virtual void childRemoved ( envire::Layer* parent, envire::Layer* child );
	virtual void itemModified ( envire::EnvironmentItem* item );
	virtual void itemAttached ( envire::EnvironmentItem* item );
	virtual void itemDetached ( envire::EnvironmentItem* item );
	virtual void frameNodeSet( envire::CartesianMap* map, envire::FrameNode* node);
	virtual void frameNodeDetached(envire::CartesianMap* map, envire::FrameNode* node);
	virtual void setRootNode(envire::FrameNode* root);
	virtual void removeRootNode(envire::FrameNode* root);
	
    protected:
	void addChild ( envire::EnvironmentItem* parent, envire::EnvironmentItem* child );    
	void removeChild ( envire::EnvironmentItem* parent, envire::EnvironmentItem* child );
	
	std::vector<EnvironmentItemVisualizer *> visualizers;
	std::map <envire::EnvironmentItem *, osg::ref_ptr<osg::Group> > environmentToNode;
	std::map <osg::Group *, envire::EnvironmentItem *> nodeToEnvironment;

	nodeCallback osgAddNode, osgRemoveNode;
};

}

#endif // ENVIREEVENTLISTENER_H
