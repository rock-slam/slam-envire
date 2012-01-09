#ifndef __ENVIEW_ENVIREVISUALIZATION__
#define __ENVIEW_ENVIREVISUALIZATION__

#include <vizkit/VizPlugin.hpp>
#include <osg/Geometry>
#include <envire/Core.hpp>

#include <boost/thread/recursive_mutex.hpp>
#include <vizkit/EnvireEventListener.hpp>
#include <vizkit/TreeViewListener.hpp>

class QTreeWidget;

namespace vizkit 
{

class EnvireVisualization : public VizPluginAdapter<envire::Environment*>
{
    Q_OBJECT
    Q_PROPERTY(bool pcl_visualizer READ isPCLVisualizerEnabled WRITE setPCLVisualizerEnabled USER true)
    Q_PROPERTY(bool mls_visualizer READ isMLSVisualizerEnabled WRITE setMLSViusalizerEnabled USER true)
    Q_PROPERTY(bool mls_visualizer_cell_color READ isMLSVisualizerCellColorEnabled WRITE setMLSViusalizerCellColorEnabled USER true)

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    EnvireVisualization();    
    ~EnvireVisualization();    

    void attachTreeWidget( QTreeWidget *treeWidget );
    bool isDirty() const;

    /** Load the environment from disk and display it */
    void load(std::string const& path);

    /** set to false if you want to manually handle the dirty flag by calling
     * setDirty() directly 
     */
    void handleDirty( bool handleDirty ) { m_handleDirty = handleDirty; }

    /** @return the currently selected item, or NULL if there is no selection
     */
    envire::EnvironmentItem* getSelectedItem() { return twl ? twl->selected : NULL; }

    void setFilter( envire::EventFilter *filter ) { eventListener->setFilter( filter ); }

    bool isPCLVisualizerEnabled();
    bool isMLSVisualizerEnabled();
    bool isMLSVisualizerCellColorEnabled();

    void setPCLVisualizerEnabled(bool value);
    void setMLSViusalizerEnabled(bool value);
    void setMLSViusalizerCellColorEnabled(bool value);

protected:
    virtual void operatorIntern( osg::Node* node, osg::NodeVisitor* nv );
    virtual void updateDataIntern( envire::Environment* const& data );

private:
    bool m_handleDirty;
    /** If true, the object pointed-to by @c env is owned by this object */
    bool m_ownsEnvironment;

    envire::Environment *env;
    boost::recursive_mutex envLock;

    std::vector<boost::shared_ptr<EnvironmentItemVisualizer> > visualizers;

    boost::shared_ptr<EnvireEventListener> eventListener;
    boost::shared_ptr<TreeViewListener> twl;
};

}
#endif 
