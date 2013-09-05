#ifndef FRAMENODEVISUALIZATION_H
#define FRAMENODEVISUALIZATION_H
#include "EnvironmentItemVisualizer.hpp"

namespace envire {

class FrameNodeVisualization: public envire::EnvironmentItemVisualizer
{
    Q_OBJECT

    Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)

    public slots:
        bool isUncertaintyShown() const {return showUncertainty;}
        void setShowUncertainty(bool enabled) {showUncertainty = enabled; emit propertyChanged("show_uncertainty");}

    public:
        FrameNodeVisualization();
	virtual osg::Group* getNodeForItem(envire::EnvironmentItem* item) const;
	virtual bool handlesItem(envire::EnvironmentItem* item) const;
	virtual void updateNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void highlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
	virtual void unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const;
        virtual void hideNode(envire::EnvironmentItem *item, osg::Group *group) const;
        virtual void unHideNode(envire::EnvironmentItem *item, osg::Group *group) const;

    protected:
        bool showUncertainty;
};

};
#endif // FRAMENODEVISUALIZATION_H
