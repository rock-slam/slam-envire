#ifndef EVIRONMENTITEMVIZALIZER_H
#define EVIRONMENTITEMVIZALIZER_H

#include <vizkit3d/Vizkit3DPlugin.hpp>

#define INVISIBLE_MASK 0x0
#define VISIBLE_MASK 0xffffffff

namespace osg {
    class Group;
}


namespace envire {
    class EnvironmentItem;
}

namespace envire {
    
class EnvironmentItemVisualizer : public vizkit3d::VizPluginBase
{
    public:
        virtual void updateMainNode(osg::Node* node) {};
	virtual bool handlesItem(envire::EnvironmentItem *item) const = 0;
	virtual osg::Group *getNodeForItem(envire::EnvironmentItem *item) const = 0;
	virtual void updateNode(envire::EnvironmentItem *item, osg::Group *group) const = 0; 
	bool equal(EnvironmentItemVisualizer const& other) const {
	    return this == &other;
	}
	virtual void highlightNode(envire::EnvironmentItem *item, osg::Group *group) const = 0; 
	virtual void unHighlightNode(envire::EnvironmentItem *item, osg::Group *group) const = 0;
        virtual void hideNode(envire::EnvironmentItem *item, osg::Group *group) const {};//= 0;
        virtual void unHideNode(envire::EnvironmentItem *item, osg::Group *group) const {};//= 0;
};

}

#endif
