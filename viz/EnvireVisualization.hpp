#ifndef __ENVIEW_ENVIREVISUALIZATION__
#define __ENVIEW_ENVIREVISUALIZATION__

#include <vizkit/VizPlugin.hpp>
#include <osg/Geometry>
#include <envire/Core.hpp>

#include <boost/thread/recursive_mutex.hpp>
#include "EnvireEventListener.hpp"
#include "TreeViewListener.hpp"

class QTreeWidget;

namespace vizkit {

class EnvireVisualization : public VizPlugin<envire::Environment*>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EnvireVisualization();    

    void attachTreeWidget( QTreeWidget *treeWidget );
    bool isDirty() const;
    
protected:
    virtual void operatorIntern( osg::Node* node, osg::NodeVisitor* nv );
    virtual void updateDataIntern( envire::Environment* const& data );

private:
    envire::Environment *env;
    boost::recursive_mutex envLock;

    std::vector<boost::shared_ptr<EnvironmentItemVisualizer> > visualizers;

    boost::shared_ptr<EnvireEventListener> eventListener;
    boost::shared_ptr<TreeViewListener> twl;
};

}
#endif 
