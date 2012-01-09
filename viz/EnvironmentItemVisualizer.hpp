#ifndef EVIRONMENTITEMVIZALIZER_H
#define EVIRONMENTITEMVIZALIZER_H

#define INVISIBLE_MASK 0x0
#define VISIBLE_MASK 0xffffffff

namespace osg {
    class Group;
}


namespace envire {
    class EnvironmentItem;
}

namespace vizkit {
    
class EnvironmentItemVisualizer 
{
    public:
        EnvironmentItemVisualizer():enabled(true){};

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

        bool isEnabled()const{return enabled;};
        void setEnabled(bool value){enabled = value;};

    private:
        bool enabled;
};

}

#endif
